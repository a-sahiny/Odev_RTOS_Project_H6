/**
  ******************************************************************************
  * @file    uart_link.c
  * @brief   USART1 byte link: RX circular DMA + IDLE -> sbUartRx, TX DMA with a
  *          task notification, the halt line by DMA (docs 13.1). No polling,
  *          no critical section (report 092).
  ******************************************************************************
  * RTOS object (one):
  *   sbUartRx     static StreamBuffer, 2048 usable bytes, trigger 1.
  *                Writer: HAL_UARTEx_RxEventCallback (USART1 / DMA2 Stream2
  *                ISR, both NVIC 5: they never preempt each other).
  *                Reader: vnetRx only, xStreamBufferReceive on the handle
  *                of UartLink_GetRxStream, <= APP_UART_RX_WAIT_MS.
  *                The kernel wakes the reader with ITS task notification
  *                index 0 (stream_buffer.c:191, :1654): vnetRx must not use
  *                that notification for anything else.
  * Task notification used (no object):
  *   index 0 of uartTx: given by HAL_UART_TxCpltCallback (ISR), taken by
  *   UartLink_SendDma (uartTx only), <= line time + APP_UART_TX_MARGIN_MS.
  * Shared memory without an RTOS object:
  *   ucUartRxDma[]  written by DMA2 Stream2, read by the RX callback.
  *   usLastPos, ulLossCount  written only by the USART1 / DMA ISRs (same
  *                  priority); ulLossCount read by vnetRx.
  */
#include "uart_link.h"

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

#include "stm32f7xx_hal.h"

#include "app_config.h"
#include "app_stats.h"

/* Defined by CubeMX in main.c; only this file uses it (K3). */
extern UART_HandleTypeDef huart1;

/* The StreamBuffer keeps one byte free (stream_buffer.c:783), so the storage
 * is one byte larger than the usable size (K4). */
static StaticStreamBuffer_t xSbUartRxBuf;
static uint8_t ucSbUartRxStorage[ APP_UART_RX_SB_BYTES + 1U ];
static StreamBufferHandle_t sbUartRx = NULL;    /* NULL = UartLink_Init not called */

static uint8_t ucUartRxDma[ APP_UART_RX_DMA_BYTES ];

static uint16_t usLastPos;          /* DMA position already copied to sbUartRx   */
static uint32_t ulLossCount;        /* loss events anywhere on the RX path        */
/* 1: the last RX event was an IDLE whose copy reached or crossed the end of
 * the ring. The TC of that same wrap may still be pending and then arrives
 * with no new bytes: not a lag (report 091). Written only by the NVIC 5
 * ISRs. */
static uint8_t ucIdleWrapped;
/* DWT cycle counts for the lag rule (report 092). A lap of the ring takes at
 * least 256 characters of line time, so a TC with no new bytes that comes
 * sooner than that after the previous IRQ cannot be a lap. Events of one
 * chain (HT and TC served by one DMA IRQ, stm32f7xx_hal_dma.c:801, :848; an
 * IDLE tail-chained to the TC) are a few us apart and count as one moment:
 * ulRefCycles is the time of the last event before the current chain.
 * Written only by the NVIC 5 ISRs (and prvStartRx before the reception
 * runs). */
static uint32_t ulLastEvtCycles;
static uint32_t ulRefCycles;

/* Shortest time the DMA can need for one lap: 256 x 16 us = 4096 us at
 * 625000 baud, 15/16 of it (3840 us) for the baud tolerance of both ends. */
#define UART_LINK_LAP_MIN_US \
    ( ( APP_UART_RX_DMA_BYTES * APP_UART_CHAR_US * 15U ) / 16U )
/* Two events closer than this belong to one chain (3 characters). */
#define UART_LINK_CHAIN_US      ( 3U * APP_UART_CHAR_US )
/* 1 once UartLink_StartRx has started the reception: from then on the error
 * callback restarts it whenever the HAL stops it. 0 before (the error
 * callback of a TX error must not start RX before pfInitialise does).
 * Written by the task once, then read by the ISR (one byte). */
static volatile uint8_t ucRxWanted;

/* The task waiting for "TX done" (uartTx). Written by UartLink_SendDma before
 * it starts the DMA, read by HAL_UART_TxCpltCallback (ISR). NULL until the
 * first send. */
static TaskHandle_t xTxWaiter = NULL;

/* When the last bit of the last line left USART1 (DWT cycles). Written by
 * HAL_UART_TxCpltCallback (ISR), read by uartTx after UartLink_SendDma gave
 * pdPASS: the M6 of a GET_IMU reply (report 091). One aligned 32-bit word:
 * a single store / load. */
static volatile uint32_t ulTxDoneCycles;

/* Longest line time plus the margin (docs 3.2 step 5): the upper bound for
 * the xWait of UartLink_SendDma. 544 B x 16 us = 8.7 ms -> 9 + 20 = 29 ms at
 * 625000 baud (report 086; 68 ms at 115200). */
#define UART_LINK_TX_WAIT_MAX_MS \
    ( ( ( ( APP_UART_TX_DMA_BYTES * APP_UART_CHAR_US ) + 999U ) / 1000U ) + APP_UART_TX_MARGIN_MS )

static void prvRxEventISR( uint32_t ulEvent, uint16_t usSize, BaseType_t * pxWoken );
static void prvRxCopyISR( uint32_t ulFrom, uint32_t ulLen, BaseType_t * pxWoken, uint32_t * pulLost );
static BaseType_t prvStartRx( void );
static void prvRestartRx( void );

_Static_assert( APP_UART_RX_DMA_BYTES <= UINT16_MAX, "HAL takes a 16-bit DMA length" );
_Static_assert( ( APP_UART_RX_DMA_BYTES % 2U ) == 0U, "HT is at exactly half the buffer" );
_Static_assert( APP_UART_RX_SB_TRIGGER <= APP_UART_RX_SB_BYTES, "trigger within the StreamBuffer" );
_Static_assert( APP_UART_TX_DMA_BYTES <= UINT16_MAX, "HAL takes a 16-bit TX length" );

/*---------------------------------------------------------------------------*/
void UartLink_Init( void )
{
    if( sbUartRx == NULL )
    {
        /* MX_USART1_UART_Init (main.c, before App_Init) linked the DMA in
         * HAL_UART_MspInit. The RX logic relies on CIRCULAR mode: in normal
         * mode the HAL would stop the DMA at the first IDLE. */
        configASSERT( huart1.hdmarx != NULL );
        configASSERT( huart1.hdmarx->Init.Mode == DMA_CIRCULAR );
        configASSERT( huart1.hdmatx != NULL );

        /* Static storage: cannot fail; the kernel itself asserts NULL
         * pointers and trigger > size (report 062, B9). */
        sbUartRx = xStreamBufferCreateStatic( sizeof( ucSbUartRxStorage ),
                                              APP_UART_RX_SB_TRIGGER,
                                              ucSbUartRxStorage,
                                              &xSbUartRxBuf );

        usLastPos = 0U;
        ulLossCount = 0U;
        ucRxWanted = 0U;
    }
}

/*---------------------------------------------------------------------------*/
BaseType_t UartLink_StartRx( void )
{
    BaseType_t xResult = pdPASS;

    configASSERT( sbUartRx != NULL );

    /* Same test the HAL makes (stm32f7xx_hal_uart_ex.c:681). READY = no
     * reception running: no RX interrupt is enabled yet (the HAL enables
     * EIE / IDLEIE / DMA at the end of the start), so nothing races with
     * this start and no critical section is needed (report 092). Busy =
     * already running: leave it alone. pfInitialise calls this from the IP
     * task's start-up only. */
    if( huart1.RxState == HAL_UART_STATE_READY )
    {
        xResult = prvStartRx();
    }

    /* After the start: from now on the error callback restarts the
     * reception by itself. */
    if( xResult == pdPASS )
    {
        ucRxWanted = 1U;
    }

    return xResult;
}

/*---------------------------------------------------------------------------*/
/* prvStartRx: reset the RX bookkeeping and start the circular DMA with IDLE  */
/* detection. Caller: UartLink_StartRx (task, once) or the                   */
/* error callback (ISR). RxState must be READY.                              */
/*---------------------------------------------------------------------------*/
static BaseType_t prvStartRx( void )
{
    BaseType_t xResult = pdPASS;

    usLastPos = 0U;
    ucIdleWrapped = 0U;
    ulLastEvtCycles = DWT->CYCCNT;
    ulRefCycles = ulLastEvtCycles;

    if( HAL_UARTEx_ReceiveToIdle_DMA( &huart1, ucUartRxDma, ( uint16_t ) APP_UART_RX_DMA_BYTES ) != HAL_OK )
    {
        xResult = pdFAIL;
    }

    return xResult;
}

/*---------------------------------------------------------------------------*/
/* HAL weak callback, shared by every UART. Context: USART1 ISR (UART error)  */
/* or DMA2 Stream2 / Stream7 ISR (DMA error, end of the DMA abort); all      */
/* NVIC 5.                                                                   */
/*---------------------------------------------------------------------------*/
void HAL_UART_ErrorCallback( UART_HandleTypeDef * huart )
{
    uint32_t ulError;

    if( huart->Instance == USART1 )
    {
        ulError = huart->ErrorCode;

        if( ( ulError & HAL_UART_ERROR_ORE ) != 0U )
        {
            Stats_Inc( eStatUartOre );
        }

        if( ( ulError & HAL_UART_ERROR_FE ) != 0U )
        {
            Stats_Inc( eStatUartFe );
        }

        if( ( ulError & HAL_UART_ERROR_NE ) != 0U )
        {
            Stats_Inc( eStatUartNe );
        }

        /* With the RX DMA running every UART error stops the reception
         * (stm32f7xx_hal_uart.c:2224-2245), and a DMA error of EITHER stream
         * stops it too (UART_DMAError :3611-3631 ends RX whenever DMAR is
         * set). So the decision is taken on RxState, not on the error code. */
        if( ( ucRxWanted != 0U ) && ( huart->RxState == HAL_UART_STATE_READY ) )
        {
            prvRestartRx();
        }
    }
}

/*---------------------------------------------------------------------------*/
/* prvRestartRx: the bytes since the last RX event are gone (the DMA was      */
/* aborted before they were copied): count one loss, then start again from  */
/* the error interrupt, with what the HAL read from the status register     */
/* (ErrorCode). Nothing polls the reception (report 092: the UART is assumed */
/* not to fail; a failed restart here is a known limit).                     */
/*---------------------------------------------------------------------------*/
static void prvRestartRx( void )
{
    ulLossCount++;
    ( void ) prvStartRx();
}

/*---------------------------------------------------------------------------*/
StreamBufferHandle_t UartLink_GetRxStream( void )
{
    configASSERT( sbUartRx != NULL );           /* UartLink_Init in App_Init */
    return sbUartRx;
}

/*---------------------------------------------------------------------------*/
uint32_t UartLink_GetLossCount( void )
{
    /* One aligned 32-bit load; written only by the USART1 / DMA ISRs. */
    return ulLossCount;
}

/*---------------------------------------------------------------------------*/
/* HAL weak callback, shared by every UART: HT / TC of the RX DMA and IDLE.   */
/* Context: USART1 or DMA2 Stream2 ISR (both NVIC 5).                         */
/*---------------------------------------------------------------------------*/
void HAL_UARTEx_RxEventCallback( UART_HandleTypeDef * huart, uint16_t Size )
{
    BaseType_t xWoken = pdFALSE;

    if( huart->Instance == USART1 )
    {
        prvRxEventISR( ( uint32_t ) HAL_UARTEx_GetRxEventType( huart ), Size, &xWoken );

        /* One xWoken for both halves of a wrapped slice: never reset in
         * between (docs 13.1). */
        portYIELD_FROM_ISR( xWoken );
    }
}

/*---------------------------------------------------------------------------*/
/* prvRxEventISR: copy the new bytes, then the lag check (one rule, B3).     */
/*---------------------------------------------------------------------------*/
static void prvRxEventISR( uint32_t ulEvent, uint16_t usSize, BaseType_t * pxWoken )
{
    const uint32_t ulSize = APP_UART_RX_DMA_BYTES;
    uint32_t ulLast = usLastPos;
    uint32_t ulPos;
    uint32_t ulLen;
    uint32_t ulLost = 0U;
    const uint32_t ulCyclesPerUs = SystemCoreClock / 1000000U;
    const uint32_t ulNow = DWT->CYCCNT;
    uint32_t ulSinceRef;

    /* Unsigned differences: right across a CYCCNT wrap (171 s at 25 MHz). */
    if( ( ulNow - ulLastEvtCycles ) >= ( UART_LINK_CHAIN_US * ulCyclesPerUs ) )
    {
        ulRefCycles = ulLastEvtCycles;      /* a new chain starts */
    }

    ulLastEvtCycles = ulNow;
    ulSinceRef = ulNow - ulRefCycles;

    /* The HAL reports 1..256; 256 (TC, or IDLE right at the end) is the
     * same place as 0. */
    configASSERT( usSize <= ulSize );
    ulPos = ( usSize >= ulSize ) ? 0U : ( uint32_t ) usSize;

    /* New bytes since the last event, going forward around the ring. */
    ulLen = ( ulPos + ulSize - ulLast ) % ulSize;

    if( ulLen > 0U )
    {
        if( ( ulLast + ulLen ) <= ulSize )
        {
            prvRxCopyISR( ulLast, ulLen, pxWoken, &ulLost );
        }
        else
        {
            /* Wrapped: end of the buffer, then its start. */
            prvRxCopyISR( ulLast, ulSize - ulLast, pxWoken, &ulLost );
            prvRxCopyISR( 0U, ulPos, pxWoken, &ulLost );
        }

        Stats_Add( eStatRxBytes, ulLen );

        if( ulLost > 0U )
        {
            Stats_Add( eStatUartRxSbOvf, ulLost );
            ulLossCount++;
        }

        if( sbUartRx != NULL )
        {
            Stats_GaugeMax( eGaugeSbUartRxMax, ( uint32_t ) xStreamBufferBytesAvailable( sbUartRx ) );
        }

        usLastPos = ( uint16_t ) ulPos;
    }

    /* Lag check, one simple rule (report 062, B3): a TC event with no new
     * bytes. TC puts the position at 0, so ulLen == 0 means the previous
     * event was at 0 as well: the DMA went once around the whole ring
     * without the HT in between being served, and 256 bytes were
     * overwritten before they were read. A lag needs the NVIC 5 ISRs late by
     * ~2 ms (half a ring at 625000); long before that other things fail.
     * Not a lag (report 091): a line that ends at or just after the end of
     * the ring with the IDLE served before the TC (both pending, USART1_IRQn
     * 37 wins over DMA2_Stream2_IRQn 58; the TC needs to be late by only one
     * character, 16 us at 625000). The IDLE already copied up to the current
     * position across the wrap, so the TC of the same wrap (it reports the
     * same position, stm32f7xx_hal_uart.c:3530) finds no new bytes. Counted
     * on its own, nothing is lost. A real lap after such an IDLE always
     * shows an HT event first (the HAL serves HT before TC in one IRQ,
     * stm32f7xx_hal_dma.c:801, :848), which clears the flag, so this check
     * misses no lap the rule found before.
     * Not a lag either (report 092, seen on the board while the host asked
     * for the IMU every 20 ms): any other empty TC that comes less than one
     * lap time after the event before its chain; the DMA cannot have written
     * 256 bytes in that time, whatever order the ISRs ran in (e.g. HT and TC
     * served in one late DMA IRQ: the HT copied up to the position, the TC
     * finds nothing new). Both cases count UART_RX_TC_AFTER_IDLE. */
    if( ( ulEvent == HAL_UART_RXEVENT_TC ) && ( ulLen == 0U ) )
    {
        if( ( ucIdleWrapped != 0U ) ||
            ( ulSinceRef < ( UART_LINK_LAP_MIN_US * ulCyclesPerUs ) ) )
        {
            Stats_Inc( eStatUartRxTcAfterIdle );
        }
        else
        {
            Stats_Inc( eStatUartRxDmaLag );
            ulLossCount++;
        }
    }

    ucIdleWrapped = ( ( ulEvent == HAL_UART_RXEVENT_IDLE ) && ( ulLen > 0U ) && ( ( ulLast + ulLen ) >= ulSize ) ) ? 1U : 0U;
}

/*---------------------------------------------------------------------------*/
/* prvRxCopyISR: one contiguous piece of the DMA buffer into sbUartRx; what  */
/* does not fit is added to *pulLost (stream mode writes what fits,          */
/* stream_buffer.c:1049).                                                    */
/*---------------------------------------------------------------------------*/
static void prvRxCopyISR( uint32_t ulFrom, uint32_t ulLen, BaseType_t * pxWoken, uint32_t * pulLost )
{
    size_t xWritten = 0U;

    if( ( ulLen > 0U ) && ( sbUartRx != NULL ) )
    {
        xWritten = xStreamBufferSendFromISR( sbUartRx, &ucUartRxDma[ ulFrom ], ( size_t ) ulLen, pxWoken );
    }

    *pulLost += ulLen - ( uint32_t ) xWritten;
}

/*---------------------------------------------------------------------------*/
BaseType_t UartLink_SendDma( const uint8_t * pucData, size_t uxLen, TickType_t xWait )
{
    BaseType_t xResult = pdFAIL;

    configASSERT( pucData != NULL );
    configASSERT( ( uxLen > 0U ) && ( uxLen <= APP_UART_TX_DMA_BYTES ) );
    configASSERT( xWait <= APP_MS( UART_LINK_TX_WAIT_MAX_MS ) );

    /* 1. A "done" left over from a line that timed out and then finished
     *    after all would end the next wait at once: drop it. */
    ( void ) ulTaskNotifyTake( pdTRUE, 0U );

    /* 2. Tell the ISR whom to wake, before the DMA can finish. */
    xTxWaiter = xTaskGetCurrentTaskHandle();

    /* 3. Start; the HAL refuses (HAL_BUSY) only if a transfer is running. */
    if( HAL_UART_Transmit_DMA( &huart1, pucData, ( uint16_t ) uxLen ) == HAL_OK )
    {
        /* 4. Wait for the last bit to leave the line (USART TC, not DMA TC). */
        if( ulTaskNotifyTake( pdTRUE, xWait ) != 0U )
        {
            xResult = pdPASS;
        }
        else
        {
            /* Stuck DMA: free the UART (gState back to READY) so the next
             * line can start. ctrlTask turns the counter into FAULT
             * UART_TX_STALL (docs 19). */
            ( void ) HAL_UART_AbortTransmit( &huart1 );
            Stats_Inc( eStatTxUartStall );
        }
    }
    else
    {
        Stats_Inc( eStatTxUartStall );
    }

    return xResult;
}

/*---------------------------------------------------------------------------*/
uint32_t UartLink_GetTxDoneCycles( void )
{
    return ulTxDoneCycles;
}

/*---------------------------------------------------------------------------*/
/* HAL weak callback, shared by every UART. Context: USART1 ISR (NVIC 5):    */
/* DMA2 Stream7 done -> TCIE -> USART TC -> UART_EndTransmit_IT              */
/* (stm32f7xx_hal_uart.c:3438, :3897-3913).                                  */
/*---------------------------------------------------------------------------*/
void HAL_UART_TxCpltCallback( UART_HandleTypeDef * huart )
{
    BaseType_t xWoken = pdFALSE;

    if( huart->Instance == USART1 )
    {
        /* M6: USART TC = the last bit is on the wire (report 091). Taken
         * before the notification, so uartTx always reads this line's. */
        ulTxDoneCycles = DWT->CYCCNT;

        if( xTxWaiter != NULL )
        {
            vTaskNotifyGiveFromISR( xTxWaiter, &xWoken );
            portYIELD_FROM_ISR( xWoken );
        }
    }
}

/*---------------------------------------------------------------------------*/
/* UartLink_HaltWrite: App_Halt only (report 092). Starts one TX DMA         */
/* transfer and returns at once: the DMA sends the line by itself, with      */
/* interrupts off and the CPU in App_Halt's endless loop; nothing waits for  */
/* the end. A line uartTx's DMA is still sending is stopped first (HAL       */
/* abort: it clears the DMA enable bit). pucData must stay valid: App_Halt's */
/* buffer is static. Longer than the DMA buffer is cut.                      */
/*---------------------------------------------------------------------------*/
void UartLink_HaltWrite( const uint8_t * pucData, size_t uxLen )
{
    if( ( pucData != NULL ) && ( uxLen > 0U ) )
    {
        if( huart1.gState != HAL_UART_STATE_READY )
        {
            ( void ) HAL_UART_AbortTransmit( &huart1 );
        }

        ( void ) HAL_UART_Transmit_DMA( &huart1, pucData,
                                        ( uint16_t ) ( ( uxLen > APP_UART_TX_DMA_BYTES ) ? APP_UART_TX_DMA_BYTES : uxLen ) );
    }
}
