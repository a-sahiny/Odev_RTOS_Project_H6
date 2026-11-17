/**
  ******************************************************************************
  * @file    uart_tx.c
  * @brief   The single TX worker uartTx and its queue (docs 3.2, 5.2, 10.1).
  ******************************************************************************
  * RTOS objects (künye: report 045):
  *   xUartTxQueue  static Queue, APP_TXQ_LEN x UartTxItem_t (12 B).
  *                 Producers, all with wait 0: the IP task (UartTx_PostFrame,
  *                 P5); ctrlTask (P6), vnetRx (P4), worker (P1) through
  *                 UartTx_PostReport. Consumer: uartTx only,
  *                 <= APP_TXQ_RECV_WAIT_MS.
  *   uartTx        static task, APP_PRIO_UART_TX (5, report 091), APP_STACK_UART_TX_WORDS.
  *                 Waits on xUartTxQueue (<= 1000 ms), on the TX done
  *                 notification (<= line time + 20 ms) and, with TXDELAY on,
  *                 in vTaskDelay (<= APP_TXDELAY_MAX_MS).
  * Task notification used (no object): index 0 of uartTx, given by the TX
  * done ISR and taken inside UartLink_SendDma (uart_link.c).
  * Shared memory without an RTOS object:
  *   usInjectedDelayMs  written by vnetRx (console), read by uartTx; atomic
  *                      16-bit store / load.
  *   ucUartTxDma[]      uartTx writes, DMA2 Stream7 reads; one line at a time.
  *   ucPadFrame[]       uartTx only.
  * Held without owning: the network buffer / report block of the item being
  * turned into a line; released before the line is sent.
  * Report 091: a GET_IMU OK reply goes to the FRONT of the queue (it waits
  * for at most the one line on the wire and nothing can get ahead of it);
  * every other item goes to the back. Its M1 -> M6 (request in -> last bit
  * out, board time only) is measured here.
  */
#include "uart_tx.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"           /* configASSERT; vTaskSuspendAll / xTaskResumeAll */
#include "queue.h"
#include "FreeRTOS_IP.h"    /* NetworkBufferDescriptor_t */
#include "NetworkBufferManagement.h"    /* vReleaseNetworkBufferAndDescriptor */

#include "stm32f7xx.h"      /* DWT->CYCCNT (U2) */

#include "app_config.h"
#include "app_stats.h"
#include "crc.h"            /* Crc32_Eth */
#include "pkt_format.h"     /* Pkt_ReadU16: big-endian, no checks */
#include "report.h"         /* Report_Release */
#include "uart_link.h"      /* UartLink_SendDma */

/* Header layout read by prvClassifyFrame (IEEE 802.3, RFC 791, RFC 768). */
#define UART_TX_ETH_HDR_BYTES       14U
#define UART_TX_ETH_TYPE_OFFSET     12U
#define UART_TX_ETH_TYPE_IPV4       0x0800U
#define UART_TX_IP_MIN_HDR_BYTES    20U
#define UART_TX_IP_PROTO_OFFSET     9U
#define UART_TX_IP_PROTO_UDP        17U
#define UART_TX_UDP_HDR_BYTES       8U

static StaticQueue_t xUartTxQueueBuf;
static uint8_t ucUartTxQueueStorage[ APP_TXQ_LEN * sizeof( UartTxItem_t ) ];
static QueueHandle_t xUartTxQueue = NULL;       /* NULL = UartTx_Init not called */

static uint16_t usInjectedDelayMs;              /* TXDELAY, 0 = off (atomic)    */

/* The last GET_IMU OK reply out: REQ_ID << 16 | M1 -> M6 us (report 091).
 * uartTx writes, ctrlTask reads; __atomic 32-bit. */
static uint32_t ulLastImuReply;

/* ucImu of prvAdmit (report 092) */
#define UART_TX_IMU_NONE        0U      /* not a GET_IMU reply                    */
#define UART_TX_IMU_OK          1U      /* GET_IMU OK reply: may go to the front  */
#define UART_TX_IMU_OTHER       2U      /* GET_IMU error reply (E_STATE, E_LEN)   */

/* The GET_IMU reply now at the front of the queue (its network buffer), NULL
 * if none (report 092). Set by prvAdmit with the scheduler suspended, cleared
 * by uartTx when it takes that item; __atomic. At most one item sits at the
 * front: a second GET_IMU reply that comes before the first left goes to the
 * back, so a host asking faster than the line can send cannot starve the
 * other command replies behind an endless row of front items. */
static void * pvImuAtFront;

/* 1: uartTx has taken an item that was not a front GET_IMU reply since the
 * last one went to the front (report 092). A GET_IMU reply goes to the front
 * only then: when the line is slower than the host asks (T06 TXDELAY), front
 * items and the others take turns instead of the others waiting forever. On
 * a normal line the queue is empty when the next request comes, and nothing
 * changes. uartTx sets it, prvAdmit clears it (scheduler suspended);
 * __atomic. */
static uint8_t ucImuMayFront = 1U;

static StaticTask_t xUartTxTcb;
static StackType_t xUartTxStack[ APP_STACK_UART_TX_WORDS ];
static TaskHandle_t xUartTxTask = NULL;         /* NULL = task not created yet  */

/* The line being sent. One owner, uartTx: it is rewritten only after
 * UartLink_SendDma has returned (DMA done or aborted). D-cache is off, so
 * the DMA reads what the CPU wrote. */
static uint8_t ucUartTxDma[ APP_UART_TX_DMA_BYTES ];
/* A frame shorter than 60 B, padded with zeros (U6). uartTx only. */
static uint8_t ucPadFrame[ APP_FRAME_MIN_BYTES ];

static const char cFramePrefix[] = "FRAME 1 ";          /* docs 2.1: always with FCS */
static const char cHexDigits[] = "0123456789ABCDEF";    /* upper case (U7) */

#define UART_TX_PREFIX_CHARS    ( sizeof( cFramePrefix ) - 1U )

static eUartTxClass_t prvClassifyFrame( const uint8_t * pucFrame, size_t uxLen );
static BaseType_t prvAdmit( UartTxItem_t * pxItem, uint8_t ucImu );
static BaseType_t prvIsImuReply( const uint8_t * pucFrame, size_t uxLen );
static void prvOnTaken( const UartTxItem_t * pxItem );
static void prvUartTxTask( void * pvParameters );
static BaseType_t prvSendFrameLine( NetworkBufferDescriptor_t * pxDesc, eUartTxClass_t eClass );
static BaseType_t prvImuReplyRx( const uint8_t * pucFrame, size_t uxLen, uint32_t * pulRxCycles, uint16_t * pusReqId );
static void prvImuReplyDone( uint32_t ulRxCycles, uint16_t usReqId );
static BaseType_t prvSendReportLine( ReportDesc_t * pxReport );
static size_t prvWriteHex( uint8_t * pucDst, const uint8_t * pucSrc, size_t uxLen );
static BaseType_t prvSendLine( size_t uxChars );
static void prvApplyInjectedDelay( void );

/* The class is also the latency offset: eLatTxqCtrl + eClass (U1; B2). */
_Static_assert( ( eLatTxqCtrl + eUartTxClassOther ) == eLatTxqOther, "class order = eLatTxq* order" );
_Static_assert( APP_TXDELAY_MAX_MS <= UINT16_MAX, "TXDELAY is kept in 16 bits" );
_Static_assert( APP_UART_TX_DMA_BYTES >= ( UART_TX_PREFIX_CHARS + ( 2U * ( APP_FRAME_MAX_BYTES + APP_FRAME_FCS_BYTES ) ) + 1U ),
                "longest frame line fits the DMA buffer" );
_Static_assert( APP_FRAME_FCS_BYTES == 4U, "FCS is one CRC-32" );
_Static_assert( ( APP_UART_CHAR_US * APP_UART_BAUD ) == 10000000U, "APP_UART_CHAR_US is exactly 10 bits at APP_UART_BAUD (U8)" );
_Static_assert( APP_IMU_REPLY_TARGET_MS < APP_IMU_REPLY_DEADLINE_MS, "target before deadline" );
_Static_assert( sizeof( "uartTx" ) <= configMAX_TASK_NAME_LEN, "task name fits" );

/*---------------------------------------------------------------------------*/
BaseType_t UartTx_Init( void )
{
    if( xUartTxQueue == NULL )
    {
        /* Static storage: NULL only for bad parameters, which the kernel
         * asserts itself (queue.c:388-394: length 0, no buffer, storage /
         * item size mismatch); all fixed here (report 062, B9). */
        xUartTxQueue = xQueueCreateStatic( APP_TXQ_LEN,
                                           sizeof( UartTxItem_t ),
                                           ucUartTxQueueStorage,
                                           &xUartTxQueueBuf );

        #if ( configQUEUE_REGISTRY_SIZE > 0 )
        {
            /* As FreeRTOS_IP.c:1002-1005 does for its event queue: lets a
             * kernel aware debugger show the queue by name. */
            vQueueAddToRegistry( xUartTxQueue, "UartTxQ" );
        }
        #endif

        __atomic_store_n( &usInjectedDelayMs, 0U, __ATOMIC_RELAXED );
    }

    if( xUartTxTask == NULL )
    {
        /* Static TCB and stack: NULL only for NULL buffers, which the kernel
         * asserts itself (tasks.c:1276-1290; B9). */
        xUartTxTask = xTaskCreateStatic( prvUartTxTask,
                                         "uartTx",
                                         APP_STACK_UART_TX_WORDS,
                                         NULL,
                                         APP_PRIO_UART_TX,
                                         xUartTxStack,
                                         &xUartTxTcb );

        /* For the stack high-water mark in the snapshot. */
        Stats_SetTaskHandle( eAppTaskUartTx, xUartTxTask );
    }

    return pdPASS;
}

/*---------------------------------------------------------------------------*/
void UartTx_SetInjectedDelay( uint16_t usMs )
{
    /* The limit is enforced where the input arrives: line_rx rejects a
     * larger TXDELAY (prvParseU32, BadArg). Here a larger value is a bug
     * (report 062, B8). */
    configASSERT( usMs <= APP_TXDELAY_MAX_MS );

    /* One writer (vnetRx), one reader (uartTx): an aligned 16-bit store is
     * never torn; atomic so the compiler cannot split or drop it. */
    __atomic_store_n( &usInjectedDelayMs, usMs, __ATOMIC_RELAXED );
}

/*---------------------------------------------------------------------------*/
uint16_t UartTx_GetInjectedDelay( void )
{
    /* The only copy of TXDELAY (report 062, B10): the console reads it here
     * for the "inj" report field. Any context, one 16-bit load. */
    return __atomic_load_n( &usInjectedDelayMs, __ATOMIC_RELAXED );
}

/*---------------------------------------------------------------------------*/
BaseType_t UartTx_PostFrame( struct xNETWORK_BUFFER * pxDesc )
{
    UartTxItem_t xItem;
    BaseType_t xResult = pdFAIL;

    configASSERT( xUartTxQueue != NULL );

    Stats_Inc( eStatIfOut );

    /* The stack never passes these: a bug, not an input (B9). */
    configASSERT( ( pxDesc != NULL ) && ( pxDesc->pucEthernetBuffer != NULL ) );

    if( ( pxDesc->xDataLength == 0U ) || ( pxDesc->xDataLength > APP_FRAME_MAX_BYTES ) )
    {
        /* An input, not a bug: an ICMP echo reply is as long as the request
         * the host sent. Longer than 256 B the line would not fit the DMA
         * buffer (U5). The buffer stays with the caller. */
        Stats_Inc( eStatTxBadLen );
    }
    else
    {
        uint32_t ulRx;
        uint16_t usReqId;
        uint8_t ucImu = UART_TX_IMU_NONE;

        xItem.eType = eUartTxFrame;
        xItem.eClass = prvClassifyFrame( pxDesc->pucEthernetBuffer, pxDesc->xDataLength );
        xItem.pvData = pxDesc;
        xItem.ulPostCycles = 0U;        /* set by prvAdmit at the send */

        if( xItem.eClass == eUartTxClassCtrl )
        {
            /* The GET_IMU OK reply has the deadline: it alone goes first
             * (report 091). Every GET_IMU reply is kept out of the slots
             * reserved for the other commands (report 092). */
            if( prvImuReplyRx( pxDesc->pucEthernetBuffer, pxDesc->xDataLength, &ulRx, &usReqId ) == pdTRUE )
            {
                ucImu = UART_TX_IMU_OK;
            }
            else if( prvIsImuReply( pxDesc->pucEthernetBuffer, pxDesc->xDataLength ) == pdTRUE )
            {
                ucImu = UART_TX_IMU_OTHER;
            }
            else
            {
                /* START / STOP / SET_RATE / GET_STATS reply */
            }
        }

        xResult = prvAdmit( &xItem, ucImu );
    }

    return xResult;
}

/*---------------------------------------------------------------------------*/
BaseType_t UartTx_PostReport( ReportDesc_t * pxReport )
{
    UartTxItem_t xItem;

    configASSERT( xUartTxQueue != NULL );

    Stats_Inc( eStatTxReportIn );

    /* report.c never hands over an empty or overlong line: a bug (B9). */
    configASSERT( ( pxReport != NULL ) && ( pxReport->usLen != 0U ) && ( pxReport->usLen <= APP_REPORT_MAX_CHARS ) );

    xItem.eType = eUartTxReport;
    xItem.eClass = eUartTxClassOther;       /* reports never use the reserve (B2) */
    xItem.pvData = pxReport;
    xItem.ulPostCycles = 0U;

    return prvAdmit( &xItem, UART_TX_IMU_NONE );
}

/*---------------------------------------------------------------------------*/
/* prvClassifyFrame: is it a command reply? (U3; report 062, B2). The       */
/* reserve needs this one fact: IPv4, UDP, source port 5000. Everything     */
/* else is Other.                                                            */
/*---------------------------------------------------------------------------*/
static eUartTxClass_t prvClassifyFrame( const uint8_t * pucFrame, size_t uxLen )
{
    eUartTxClass_t eClass = eUartTxClassOther;
    size_t uxIpHdrBytes;

    if( ( uxLen >= ( UART_TX_ETH_HDR_BYTES + UART_TX_IP_MIN_HDR_BYTES ) ) &&
        ( Pkt_ReadU16( &pucFrame[ UART_TX_ETH_TYPE_OFFSET ] ) == UART_TX_ETH_TYPE_IPV4 ) &&
        ( ( pucFrame[ UART_TX_ETH_HDR_BYTES ] >> 4 ) == 4U ) &&
        ( pucFrame[ UART_TX_ETH_HDR_BYTES + UART_TX_IP_PROTO_OFFSET ] == UART_TX_IP_PROTO_UDP ) )
    {
        /* IHL in 32-bit words: the UDP header follows any IP options; its
         * first field is the source port. */
        uxIpHdrBytes = ( size_t ) ( pucFrame[ UART_TX_ETH_HDR_BYTES ] & 0x0FU ) * 4U;

        if( ( uxIpHdrBytes >= UART_TX_IP_MIN_HDR_BYTES ) &&
            ( uxLen >= ( UART_TX_ETH_HDR_BYTES + uxIpHdrBytes + 2U ) ) &&
            ( Pkt_ReadU16( &pucFrame[ UART_TX_ETH_HDR_BYTES + uxIpHdrBytes ] ) == APP_UDP_PORT_CRIT ) )
        {
            eClass = eUartTxClassCtrl;
        }
    }

    return eClass;
}

/*---------------------------------------------------------------------------*/
/* prvAdmit: queue the item if its class may use the free slots (docs 10.1). */
/* xFront (a GET_IMU OK reply, report 091): to the front; uartTx sends it    */
/* right after the line on the wire, nothing else can get ahead of it. Every */
/* other item, other command replies too, to the back. Report 092: only ONE */
/* item at the front at a time (pvImuAtFront), and the next one only after   */
/* uartTx has taken some other item (ucImuMayFront): a second GET_IMU reply  */
/* waits at the back like the other replies.                                 */
/* Four tasks post non-control items (P6, P5, P4, P1). With the scheduler    */
/* suspended none of them can run between the check and the send, so the    */
/* last APP_TXQ_RESERVE slots stay for control frames (U4-B). Interrupts     */
/* stay on; no ISR posts to this queue. A send with wait 0 is allowed while  */
/* suspended (queue.c:965 asserts only for a wait); a task it wakes is put   */
/* on xPendingReadyList and moved at xTaskResumeAll (tasks.c:4097).          */
/*---------------------------------------------------------------------------*/
static BaseType_t prvAdmit( UartTxItem_t * pxItem, uint8_t ucImu )
{
    eStatId_t eOutcome;
    UBaseType_t uxSpaces;
    BaseType_t xSent;
    BaseType_t xToFront;
    BaseType_t xMayUseReserve;

    vTaskSuspendAll();
    {
        uxSpaces = uxQueueSpacesAvailable( xUartTxQueue );

        /* Report 092: only the one GET_IMU OK reply that goes to the front
         * may use the last slots; every other GET_IMU reply (a second OK one
         * while the front is taken, an E_STATE one) is an ordinary item for
         * the reserve. So a host asking for the IMU faster than the line can
         * send never uses up the slots kept for START / STOP / SET_RATE /
         * GET_STATS replies (T06 with TXDELAY and the GUI watching). */
        xToFront = ( ( ucImu == UART_TX_IMU_OK ) &&
                     ( __atomic_load_n( &pvImuAtFront, __ATOMIC_RELAXED ) == NULL ) &&
                     ( __atomic_load_n( &ucImuMayFront, __ATOMIC_RELAXED ) != 0U ) ) ? pdTRUE : pdFALSE;
        xMayUseReserve = ( ( pxItem->eClass == eUartTxClassCtrl ) && ( ( ucImu == UART_TX_IMU_NONE ) || ( xToFront == pdTRUE ) ) ) ? pdTRUE : pdFALSE;

        if( uxSpaces == 0U )
        {
            eOutcome = eStatTxqFull;
        }
        else if( ( xMayUseReserve == pdFALSE ) && ( uxSpaces <= APP_TXQ_RESERVE ) )
        {
            eOutcome = eStatTxqReserve;
        }
        else
        {
            pxItem->ulPostCycles = DWT->CYCCNT;     /* M4 (docs 8.1) */

            /* Cannot fail: a slot is free and no other task runs (B9). */
            if( xToFront == pdTRUE )
            {
                __atomic_store_n( &pvImuAtFront, pxItem->pvData, __ATOMIC_RELAXED );
                __atomic_store_n( &ucImuMayFront, 0U, __ATOMIC_RELAXED );
                xSent = xQueueSendToFront( xUartTxQueue, pxItem, 0U );
            }
            else
            {
                xSent = xQueueSendToBack( xUartTxQueue, pxItem, 0U );
            }

            configASSERT( xSent == pdPASS );
            eOutcome = eStatTxqOk;
        }
    }
    ( void ) xTaskResumeAll();

    /* Counted after the resume: keeps the suspended part short. */
    Stats_Inc( eOutcome );

    if( eOutcome == eStatTxqOk )
    {
        Stats_GaugeMin( eGaugeUartTxqMinFree, ( uint32_t ) ( uxSpaces - 1U ) );
    }

    return ( eOutcome == eStatTxqOk ) ? pdPASS : pdFAIL;
}

/*---------------------------------------------------------------------------*/
/* prvOnTaken: uartTx took pxItem from the queue (report 092). The front    */
/* GET_IMU reply has left: the front is free again. Any other item had its  */
/* turn: the next GET_IMU reply may go to the front again. Only uartTx      */
/* calls it; prvAdmit reads both inside its scheduler suspend.              */
/*---------------------------------------------------------------------------*/
static void prvOnTaken( const UartTxItem_t * pxItem )
{
    if( pxItem->pvData == __atomic_load_n( &pvImuAtFront, __ATOMIC_RELAXED ) )
    {
        __atomic_store_n( &pvImuAtFront, NULL, __ATOMIC_RELAXED );
    }
    else
    {
        __atomic_store_n( &ucImuMayFront, 1U, __ATOMIC_RELAXED );
    }
}

/*---------------------------------------------------------------------------*/
/* prvUartTxTask: the only TX worker. One item = one line. The buffer or    */
/* block goes back before the line is sent, so a slow UART holds at most    */
/* this one item (docs 9: "uartTx elinde 1").                               */
/*---------------------------------------------------------------------------*/
static void prvUartTxTask( void * pvParameters )
{
    UartTxItem_t xItem;
    BaseType_t xSent;
    uint32_t ulTakenCycles;

    ( void ) pvParameters;

    for( ; ; )
    {
        /* Timeout: nothing to do, loop again (a point for a future
         * watchdog kick). */
        if( xQueueReceive( xUartTxQueue, &xItem, pdMS_TO_TICKS( APP_TXQ_RECV_WAIT_MS ) ) == pdPASS )
        {
            /* M5 (docs 8.1): TXQ_WAIT = M5 - M4 per class. Unsigned
             * difference: right across a CYCCNT wrap (171 s at 25 MHz). */
            ulTakenCycles = DWT->CYCCNT;
            prvOnTaken( &xItem );
            configASSERT( xItem.eClass < eUartTxClassCount );

            /* uartTx is the only writer of eLatTxqCtrl / eLatTxqOther (B2). */
            Stats_LatencyAdd( ( eLatId_t ) ( ( uint32_t ) eLatTxqCtrl + ( uint32_t ) xItem.eClass ),
                              ulTakenCycles - xItem.ulPostCycles );

            /* After M5 (U9): this item's own wait does not include the
             * injected delay, the items behind it do. */
            prvApplyInjectedDelay();

            if( xItem.eType == eUartTxFrame )
            {
                xSent = prvSendFrameLine( ( NetworkBufferDescriptor_t * ) xItem.pvData, xItem.eClass );
            }
            else
            {
                /* Only the two Post functions fill the queue. */
                configASSERT( xItem.eType == eUartTxReport );
                xSent = prvSendReportLine( ( ReportDesc_t * ) xItem.pvData );
            }

            /* pdFAIL: UartLink_SendDma counted TX_UART_STALL; ctrlTask
             * decides about FAULT (docs 19). */
            if( xSent == pdPASS )
            {
                Stats_Inc( eStatTxLines );
            }
        }
    }
}

/*---------------------------------------------------------------------------*/
/* prvSendFrameLine: "FRAME 1 <hex frame><hex FCS>\n" (docs 2.1, 13.4).     */
/* Frames shorter than 60 B are padded with zeros, as an Ethernet MAC would */
/* (U6); the network buffer itself is not touched. The FCS is sent low byte */
/* first, as on the wire (crc.h).                                           */
/*---------------------------------------------------------------------------*/
static BaseType_t prvSendFrameLine( NetworkBufferDescriptor_t * pxDesc, eUartTxClass_t eClass )
{
    const uint8_t * pucFrame = pxDesc->pucEthernetBuffer;
    size_t uxLen = pxDesc->xDataLength;     /* 1..256, checked by PostFrame */
    size_t uxPos;
    uint32_t ulFcs;
    uint8_t ucFcs[ APP_FRAME_FCS_BYTES ];
    uint32_t ulRxCycles = 0U;
    uint16_t usReqId = 0U;
    BaseType_t xImuReply = pdFALSE;
    BaseType_t xSent;

    /* Read before the buffer goes back (report 086). */
    if( eClass == eUartTxClassCtrl )
    {
        xImuReply = prvImuReplyRx( pucFrame, uxLen, &ulRxCycles, &usReqId );
    }

    if( uxLen < APP_FRAME_MIN_BYTES )
    {
        ( void ) memcpy( ucPadFrame, pucFrame, uxLen );
        ( void ) memset( &ucPadFrame[ uxLen ], 0, APP_FRAME_MIN_BYTES - uxLen );
        pucFrame = ucPadFrame;
        uxLen = APP_FRAME_MIN_BYTES;
    }

    ulFcs = Crc32_Eth( pucFrame, uxLen );
    ucFcs[ 0 ] = ( uint8_t ) ulFcs;
    ucFcs[ 1 ] = ( uint8_t ) ( ulFcs >> 8 );
    ucFcs[ 2 ] = ( uint8_t ) ( ulFcs >> 16 );
    ucFcs[ 3 ] = ( uint8_t ) ( ulFcs >> 24 );

    ( void ) memcpy( ucUartTxDma, cFramePrefix, UART_TX_PREFIX_CHARS );
    uxPos = UART_TX_PREFIX_CHARS;
    uxPos += prvWriteHex( &ucUartTxDma[ uxPos ], pucFrame, uxLen );
    uxPos += prvWriteHex( &ucUartTxDma[ uxPos ], ucFcs, sizeof( ucFcs ) );
    ucUartTxDma[ uxPos ] = ( uint8_t ) '\n';
    uxPos++;

    /* The line is in the DMA buffer: the network buffer goes back to the
     * pool now, not after the UART has sent it (docs 9.1). */
    vReleaseNetworkBufferAndDescriptor( pxDesc );

    xSent = prvSendLine( uxPos );

    if( ( xSent == pdPASS ) && ( xImuReply == pdTRUE ) )
    {
        prvImuReplyDone( ulRxCycles, usReqId );
    }

    return xSent;
}

/*---------------------------------------------------------------------------*/
/* prvImuReplyRx: is this frame a GET_IMU OK reply? Then its t_rx (the       */
/* request's M1) and REQ_ID. prvClassifyFrame already found IPv4 / UDP /     */
/* source port 5000; cmd.c built the packet, so its CRC is not checked here. */
/*---------------------------------------------------------------------------*/
static BaseType_t prvImuReplyRx( const uint8_t * pucFrame, size_t uxLen, uint32_t * pulRxCycles, uint16_t * pusReqId )
{
    BaseType_t xFound = pdFALSE;
    const size_t uxPkt = UART_TX_ETH_HDR_BYTES + ( ( size_t ) ( pucFrame[ UART_TX_ETH_HDR_BYTES ] & 0x0FU ) * 4U ) + UART_TX_UDP_HDR_BYTES;
    const uint8_t * pucPkt = &pucFrame[ uxPkt ];

    if( ( uxLen >= ( uxPkt + APP_PKT_OVERHEAD_BYTES + APP_RESP_COMMON_BYTES + APP_LEN_IMU_REPLY ) ) &&
        ( pucPkt[ 0 ] == APP_PKT_SOF ) &&
        ( pucPkt[ 2 ] == APP_TYPE_CMD_GET_IMU ) &&
        ( pucPkt[ 3 ] == APP_PKT_FLAG_RESP ) &&
        ( pucPkt[ 8 ] == ( APP_RESP_COMMON_BYTES + APP_LEN_IMU_REPLY ) ) )
    {
        *pulRxCycles = Pkt_ReadU32( &pucPkt[ APP_PKT_HEADER_BYTES + APP_RESP_COMMON_BYTES + APP_IMU_REPLY_RX_OFFSET ] );
        *pusReqId = Pkt_ReadU16( &pucPkt[ 6 ] );        /* REQ_ID (docs 15.1) */
        xFound = pdTRUE;
    }

    return xFound;
}

/*---------------------------------------------------------------------------*/
/* prvIsImuReply: any reply to GET_IMU (OK or error), report 092. Same       */
/* reading as prvImuReplyRx: prvClassifyFrame found IPv4 / UDP / port 5000.  */
/*---------------------------------------------------------------------------*/
static BaseType_t prvIsImuReply( const uint8_t * pucFrame, size_t uxLen )
{
    const size_t uxPkt = UART_TX_ETH_HDR_BYTES + ( ( size_t ) ( pucFrame[ UART_TX_ETH_HDR_BYTES ] & 0x0FU ) * 4U ) + UART_TX_UDP_HDR_BYTES;
    const uint8_t * pucPkt = &pucFrame[ uxPkt ];

    return ( ( uxLen >= ( uxPkt + APP_PKT_HEADER_BYTES ) ) &&
             ( pucPkt[ 0 ] == APP_PKT_SOF ) &&
             ( pucPkt[ 2 ] == APP_TYPE_CMD_GET_IMU ) &&
             ( ( pucPkt[ 3 ] & APP_PKT_FLAG_RESP ) != 0U ) ) ? pdTRUE : pdFALSE;
}

/*---------------------------------------------------------------------------*/
/* prvImuReplyDone: M6 = the instant the TX done interrupt saw the last bit */
/* leave the UART (report 091). M1 -> M6 into the histogram; over the target */
/* and over the deadline are counted (report 086 G1); the value goes into    */
/* the next GET_IMU reply. Board time only: no USB / Windows part. Unsigned  */
/* difference: right across a CYCCNT wrap.                                   */
/*---------------------------------------------------------------------------*/
static void prvImuReplyDone( uint32_t ulRxCycles, uint16_t usReqId )
{
    const uint32_t ulCycles = UartLink_GetTxDoneCycles() - ulRxCycles;
    const uint32_t ulCyclesPerMs = ( uint32_t ) ( configCPU_CLOCK_HZ / 1000UL );
    uint32_t ulUs = ulCycles / ( uint32_t ) ( configCPU_CLOCK_HZ / 1000000UL );

    /* uartTx is the only writer of eLatImuReply. */
    Stats_LatencyAdd( eLatImuReply, ulCycles );

    if( ulCycles > ( APP_IMU_REPLY_TARGET_MS * ulCyclesPerMs ) )
    {
        Stats_Inc( eStatImuReplyOver10 );
    }

    if( ulCycles > ( APP_IMU_REPLY_DEADLINE_MS * ulCyclesPerMs ) )
    {
        Stats_Inc( eStatImuReplyMiss );
    }

    if( ulUs > 0xFFFFU )
    {
        ulUs = 0xFFFFU;
    }

    __atomic_store_n( &ulLastImuReply, ( ( uint32_t ) usReqId << 16 ) | ulUs, __ATOMIC_RELAXED );
}

/*---------------------------------------------------------------------------*/
uint32_t UartTx_GetLastImuReply( void )
{
    return __atomic_load_n( &ulLastImuReply, __ATOMIC_RELAXED );
}

/*---------------------------------------------------------------------------*/
/* prvSendReportLine: the JSON text and '\n'; the block goes back to        */
/* xReportPool before the line is sent (docs 5.2).                          */
/*---------------------------------------------------------------------------*/
static BaseType_t prvSendReportLine( ReportDesc_t * pxReport )
{
    size_t uxLen = pxReport->usLen;         /* 1..320, checked by PostReport */

    ( void ) memcpy( ucUartTxDma, pxReport->cText, uxLen );
    ucUartTxDma[ uxLen ] = ( uint8_t ) '\n';

    Report_Release( pxReport );

    return prvSendLine( uxLen + 1U );
}

/*---------------------------------------------------------------------------*/
/* prvWriteHex: two upper-case hex digits per byte (U7); returns the number */
/* of characters written.                                                    */
/*---------------------------------------------------------------------------*/
static size_t prvWriteHex( uint8_t * pucDst, const uint8_t * pucSrc, size_t uxLen )
{
    size_t i;

    for( i = 0U; i < uxLen; i++ )
    {
        pucDst[ 2U * i ] = ( uint8_t ) cHexDigits[ pucSrc[ i ] >> 4 ];
        pucDst[ ( 2U * i ) + 1U ] = ( uint8_t ) cHexDigits[ pucSrc[ i ] & 0x0FU ];
    }

    return 2U * uxLen;
}

/*---------------------------------------------------------------------------*/
/* prvSendLine: send ucUartTxDma[ 0 .. uxChars ) and wait for the end, at   */
/* most the line time + APP_UART_TX_MARGIN_MS (U8): ceil( n x 16 us ) ms +  */
/* 20 ms at 625000 baud; 544 characters -> 9 + 20 = 29 ms, the limit        */
/* UartLink_SendDma asserts.                                                 */
/*---------------------------------------------------------------------------*/
static BaseType_t prvSendLine( size_t uxChars )
{
    uint32_t ulMs = ( ( ( ( uint32_t ) uxChars * APP_UART_CHAR_US ) + 999U ) / 1000U ) + APP_UART_TX_MARGIN_MS;

    return UartLink_SendDma( ucUartTxDma, uxChars, pdMS_TO_TICKS( ulMs ) );
}

/*---------------------------------------------------------------------------*/
/* prvApplyInjectedDelay: TXDELAY (docs 13.5, T06). uartTx keeps the item   */
/* while it waits, so the queue fills up behind it: the artificial slow     */
/* consumer T06 asks for. Read once per item: a new TXDELAY applies from    */
/* the next item on.                                                        */
/*---------------------------------------------------------------------------*/
static void prvApplyInjectedDelay( void )
{
    uint16_t usMs = __atomic_load_n( &usInjectedDelayMs, __ATOMIC_RELAXED );

    if( usMs != 0U )
    {
        vTaskDelay( pdMS_TO_TICKS( usMs ) );
    }
}
