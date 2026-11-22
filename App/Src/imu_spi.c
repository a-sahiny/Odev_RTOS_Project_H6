/**
  ******************************************************************************
  * @file    imu_spi.c
  * @brief   The only owner of SPI2 and the IMU (WHO_AM_I 0x74, 6 axes): reset
  *          and configuration (SPI2 DMA, <= 1 MHz), DRDY on EXTI9_5, one sample
  *          with SPI2 DMA (3.375 MHz). docs 3.3, D18. Künye: report 076.
  *          Every transfer is DMA + task notification, none polled (report 092).
  ******************************************************************************
  * Start-up order and register values follow RTOS_IMU/App/Src/app_imu.c
  * (:215-286) without the magnetometer part (the module has none, K1).
  *
  * RTOS objects: none. ISR -> task: meas's task notification (index 0,
  * eSetBits): IMU_BIT_DRDY (EXTI9_5, NVIC 6), IMU_BIT_SPI_DONE /
  * IMU_BIT_SPI_ERROR (DMA1 Stream3/4, NVIC 5).
  */
#include "imu_spi.h"

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"           /* MPU_CS / MPU_DRDY pins, HAL */
#include "app_config.h"
#include "app_stats.h"
#include "report.h"         /* Report_ImuRate */

extern SPI_HandleTypeDef hspi2;

/* Register map (MPU-9250/9255 register map, shared by MPU-6515) */
#define IMU_REG_SMPLRT_DIV      0x19U
#define IMU_REG_CONFIG          0x1AU
#define IMU_REG_GYRO_CONFIG     0x1BU
#define IMU_REG_ACCEL_CONFIG    0x1CU
#define IMU_REG_ACCEL_CONFIG2   0x1DU
#define IMU_REG_INT_PIN_CFG     0x37U
#define IMU_REG_INT_ENABLE      0x38U
#define IMU_REG_ACCEL_XOUT_H    0x3BU
#define IMU_REG_USER_CTRL       0x6AU
#define IMU_REG_PWR_MGMT_1      0x6BU
#define IMU_REG_PWR_MGMT_2      0x6CU
#define IMU_REG_WHO_AM_I        0x75U
#define IMU_READ_FLAG           0x80U

/* Values (app_imu.c:105-121): +-1000 dps, +-4 g. Rate and DLPF: xImuRates. */
#define IMU_PWR_RESET           0x80U
#define IMU_PWR_CLKSEL_PLL      0x01U
#define IMU_USER_I2C_IF_DIS     0x10U   /* SPI only                              */
#define IMU_INTERNAL_HZ         1000U   /* DLPF_CFG 1..6: internal rate 1 kHz    */
#define IMU_CFG_GYRO_FS         0x10U   /* +-1000 dps -> 32.8 LSB/dps            */
#define IMU_CFG_ACCEL_FS        0x08U   /* +-4 g -> 8192 LSB/g                   */
#define IMU_CFG_INT_PIN         0x00U   /* active high, push-pull, 50 us pulse   */
#define IMU_CFG_INT_ENABLE      0x01U   /* RAW_RDY_EN                            */

#define IMU_RESET_WAIT_MS       100U
#define IMU_CLOCK_WAIT_MS       10U
#define IMU_REG_TIMEOUT_MS      5U      /* one 2-byte DMA register transfer      */

/* One burst: the address byte, then accel (6), temperature (2), gyro (6). */
#define IMU_BURST_BYTES         14U
#define IMU_XFER_BYTES          ( 1U + IMU_BURST_BYTES )

/* meas's notification value (index 0, eSetBits), app_imu.c:8-12 */
#define IMU_BIT_DRDY            0x01UL
#define IMU_BIT_SPI_DONE        0x02UL
#define IMU_BIT_SPI_ERROR       0x04UL
#define IMU_BITS_SPI            ( IMU_BIT_SPI_DONE | IMU_BIT_SPI_ERROR )

_Static_assert( ( IMU_REG_ACCEL_XOUT_H + IMU_BURST_BYTES - 1U ) == 0x48U, "the burst ends at GYRO_ZOUT_L" );

/* One entry per SET_RATE rate. SMPLRT_DIV = 1 kHz / rate - 1. The DLPF
 * index goes into CONFIG (gyro) and ACCEL_CONFIG2 (accel, A_DLPF_CFG with
 * ACCEL_FCHOICE_B = 0): its bandwidth is at most about half the rate, so
 * nothing above the Nyquist frequency folds into the samples (MPU-6500
 * register map: gyro / accel bandwidth 2: 92 / 99 Hz, 3: 41 / 44.8 Hz,
 * 4: 20 / 21.2 Hz, 5: 10 / 10.2 Hz, 6: 5 / 5.05 Hz). A lower bandwidth
 * also delays the signal more (5 Hz: ~19 ms); that is inside the IMU,
 * before DRDY, so the TELEM latency does not include it. */
typedef struct xIMU_RATE
{
    uint16_t usHz;
    uint8_t ucDlpf;
} ImuRate_t;

static const ImuRate_t xImuRates[] =
{
    {  10U, 0x06U },
    {  20U, 0x05U },
    {  50U, 0x04U },
    { 100U, 0x03U },    /* the values before SET_RATE changed the IMU rate */
    { 200U, 0x02U },
};

_Static_assert( ( sizeof( xImuRates ) / sizeof( xImuRates[ 0 ] ) ) == APP_RATE_COUNT, "one entry per SET_RATE rate" );

/* The task the ISRs wake; written by Imu_Init before DRDY is enabled. */
static TaskHandle_t xMeasTask = NULL;

/* The tick and the DWT cycle count of the last DRDY (ISR writes, meas
 * reads; one aligned word each). */
static volatile TickType_t xDrdyTick;
static volatile uint32_t ulDrdyCycles;

/* DMA buffers. D-cache is off: no cache maintenance, no MPU region. Byte 0
 * of the TX buffer is the address; byte 1 the value of a register write,
 * set back to 0 right after it (a burst read sends zeros). */
static uint8_t ucSpiTx[ IMU_XFER_BYTES ];
static uint8_t ucSpiRx[ IMU_XFER_BYTES ];

static BaseType_t prvWaitBits( uint32_t ulBits, TickType_t xTimeout );
static BaseType_t prvTransfer( size_t uxLen, TickType_t xTimeout );
static void prvUnpack( Sample_t * pxOut );
static void prvCsLow( void );
static void prvCsHigh( void );
static void prvSetPrescaler( uint32_t ulPrescaler );
static void prvWriteReg( uint8_t ucReg, uint8_t ucValue );
static uint8_t prvReadReg( uint8_t ucReg );
static BaseType_t prvWhoAmIOk( uint8_t ucValue );
static BaseType_t prvWriteRate( uint16_t usRateHz );

/*---------------------------------------------------------------------------*/
/* Imu_Init: app_imu.c:215-286 (K1). reset -> 100 ms -> SPI only -> PLL      */
/* clock -> 10 ms -> WHO_AM_I -> configuration -> read-back -> DRDY on ->   */
/* read speed (K7). The rate and the DLPF come from xImuRates.              */
/*---------------------------------------------------------------------------*/
BaseType_t Imu_Init( TaskHandle_t xNotifyTask, uint16_t usRateHz )
{
    configASSERT( xNotifyTask != NULL );
    xMeasTask = xNotifyTask;

    prvCsHigh();
    prvSetPrescaler( APP_IMU_SPI_PRESC_CONFIG );

    prvWriteReg( IMU_REG_PWR_MGMT_1, IMU_PWR_RESET );
    vTaskDelay( APP_MS( IMU_RESET_WAIT_MS ) );

    /* I2C interface off first: the part then listens to SPI only. */
    prvWriteReg( IMU_REG_USER_CTRL, IMU_USER_I2C_IF_DIS );
    prvWriteReg( IMU_REG_PWR_MGMT_1, IMU_PWR_CLKSEL_PLL );
    vTaskDelay( APP_MS( IMU_CLOCK_WAIT_MS ) );

    if( prvWhoAmIOk( prvReadReg( IMU_REG_WHO_AM_I ) ) == pdFALSE )
    {
        Stats_Inc( eStatImuWhoAmI );        /* K2 */
        return pdFAIL;
    }

    prvWriteReg( IMU_REG_PWR_MGMT_2, 0x00U );     /* all axes on */
    prvWriteReg( IMU_REG_GYRO_CONFIG, IMU_CFG_GYRO_FS );
    prvWriteReg( IMU_REG_ACCEL_CONFIG, IMU_CFG_ACCEL_FS );

    /* Read-back (app_imu.c:272-277): the writes are not checked one by one;
     * a wrong value here means the SPI link or the part is bad. */
    if( ( prvReadReg( IMU_REG_GYRO_CONFIG ) != IMU_CFG_GYRO_FS ) ||
        ( prvReadReg( IMU_REG_ACCEL_CONFIG ) != IMU_CFG_ACCEL_FS ) ||
        ( prvWriteRate( usRateHz ) != pdPASS ) )
    {
        Stats_Inc( eStatImuWhoAmI );        /* K2 */
        return pdFAIL;
    }

    prvWriteReg( IMU_REG_INT_PIN_CFG, IMU_CFG_INT_PIN );
    prvWriteReg( IMU_REG_INT_ENABLE, IMU_CFG_INT_ENABLE );

    prvSetPrescaler( APP_IMU_SPI_PRESC_DATA );

    return pdPASS;
}

/*---------------------------------------------------------------------------*/
/* Imu_SetRate: SET_RATE on a running IMU. The register writes run at the    */
/* configuration speed (<= 1 MHz), as in Imu_Init; the read speed again      */
/* after them. A DRDY during the writes is dropped by the caller.            */
/*---------------------------------------------------------------------------*/
BaseType_t Imu_SetRate( uint16_t usRateHz )
{
    BaseType_t xResult;

    prvSetPrescaler( APP_IMU_SPI_PRESC_CONFIG );
    xResult = prvWriteRate( usRateHz );
    prvSetPrescaler( APP_IMU_SPI_PRESC_DATA );

    if( xResult != pdPASS )
    {
        Stats_Inc( eStatImuWhoAmI );        /* K2: the SPI link or the part is bad */
    }

    return xResult;
}

/*---------------------------------------------------------------------------*/
/* prvWriteRate: SMPLRT_DIV, CONFIG (gyro DLPF), ACCEL_CONFIG2 (accel DLPF)  */
/* for usRateHz, then read back. cmd.c lets only APP_RATE_LIST_HZ through.   */
/* Seen on the board (22.11.2026, imu_rate reports): on a running IMU,       */
/* CONFIG and ACCEL_CONFIG2 read back at once, but SMPLRT_DIV still reads    */
/* the OLD divider right after the write (w 4 r 9, w 19 r 4, w 9 r 19); the  */
/* new one is in force a little later (the next attempt read it, and DRDY    */
/* came at the new rate). So SMPLRT_DIV is read again every tick until it    */
/* matches, at most one period of the slowest rate + margin. Every rate      */
/* write sends an imu_rate report with the time that took.                   */
/*---------------------------------------------------------------------------*/
static BaseType_t prvWriteRate( uint16_t usRateHz )
{
    const ImuRate_t * pxRate = NULL;
    uint8_t ucDiv;
    size_t x;

    for( x = 0U; x < ( sizeof( xImuRates ) / sizeof( xImuRates[ 0 ] ) ); x++ )
    {
        if( xImuRates[ x ].usHz == usRateHz )
        {
            pxRate = &xImuRates[ x ];
            break;
        }
    }

    configASSERT( pxRate != NULL );         /* a bug, not an input (B8) */

    ucDiv = ( uint8_t ) ( ( IMU_INTERNAL_HZ / pxRate->usHz ) - 1U );

    prvWriteReg( IMU_REG_SMPLRT_DIV, ucDiv );
    prvWriteReg( IMU_REG_CONFIG, pxRate->ucDlpf );
    prvWriteReg( IMU_REG_ACCEL_CONFIG2, pxRate->ucDlpf );

    {
        const uint8_t ucWritten[ 3 ] = { ucDiv, pxRate->ucDlpf, pxRate->ucDlpf };
        uint8_t ucRead[ 3 ];
        const TickType_t xStart = xTaskGetTickCount();
        TickType_t xWaited;
        BaseType_t xOk;

        ucRead[ 1 ] = prvReadReg( IMU_REG_CONFIG );
        ucRead[ 2 ] = prvReadReg( IMU_REG_ACCEL_CONFIG2 );

        for( ; ; )
        {
            ucRead[ 0 ] = prvReadReg( IMU_REG_SMPLRT_DIV );
            xWaited = xTaskGetTickCount() - xStart;

            if( ( ucRead[ 0 ] == ucDiv ) || ( xWaited >= APP_MS( APP_IMU_DRDY_TIMEOUT_MAX_MS ) ) )
            {
                break;
            }

            vTaskDelay( 1U );
        }

        xOk = ( memcmp( ucWritten, ucRead, sizeof( ucRead ) ) == 0 ) ? pdPASS : pdFAIL;
        Report_ImuRate( usRateHz, ucWritten, ucRead, ( uint32_t ) xWaited, xOk );

        return xOk;
    }
}

/*---------------------------------------------------------------------------*/
/* Imu_DiscardStaleDrdy: app_imu.c:294-299. The IMU kept raising DRDY with  */
/* its old configuration while Imu_Init ran (a board reset does not reset   */
/* it): the first sample must come after the new configuration.             */
/*---------------------------------------------------------------------------*/
void Imu_DiscardStaleDrdy( void )
{
    ( void ) ulTaskNotifyValueClear( NULL, IMU_BIT_DRDY );
}

/*---------------------------------------------------------------------------*/
BaseType_t Imu_WaitDrdy( TickType_t xTimeout )
{
    configASSERT( xTimeout <= APP_MS( APP_IMU_DRDY_TIMEOUT_MAX_MS ) );

    if( prvWaitBits( IMU_BIT_DRDY, xTimeout ) == pdFAIL )
    {
        Stats_Inc( eStatImuDrdyTimeout );
        return pdFAIL;
    }

    return pdPASS;
}

/*---------------------------------------------------------------------------*/
/* Imu_ReadSample: app_imu.c:356-404 with a 14-byte burst (K1). CS stays low */
/* from the start of the DMA transfer until it is done or aborted.          */
/*---------------------------------------------------------------------------*/
BaseType_t Imu_ReadSample( Sample_t * pxOut, TickType_t xTimeout )
{
    configASSERT( pxOut != NULL );
    configASSERT( xTimeout <= APP_MS( APP_IMU_SPI_TIMEOUT_MS ) );

    ucSpiTx[ 0 ] = ( uint8_t ) ( IMU_REG_ACCEL_XOUT_H | IMU_READ_FLAG );

    if( prvTransfer( IMU_XFER_BYTES, xTimeout ) != pdPASS )
    {
        /* timeout, DMA error, or both bits (an error after done) */
        Stats_Inc( eStatImuSpiTimeout );            /* K3 */
        return pdFAIL;
    }

    prvUnpack( pxOut );
    pxOut->ulTick = xDrdyTick;
    pxOut->ulDrdyCycles = ulDrdyCycles;

    return pdPASS;
}

/*---------------------------------------------------------------------------*/
/* prvTransfer: one SPI DMA transfer of ucSpiTx -> ucSpiRx, uxLen bytes, CS  */
/* low from the start until it is done or aborted (app_imu.c:356-404). The   */
/* task sleeps on its notification until HAL_SPI_TxRxCpltCallback or         */
/* HAL_SPI_ErrorCallback sets a bit (no polling; report 092): the sample     */
/* burst and every register access of Imu_Init use it.                       */
/*---------------------------------------------------------------------------*/
static BaseType_t prvTransfer( size_t uxLen, TickType_t xTimeout )
{
    uint32_t ulBits = 0U;
    TimeOut_t xTimeOut;
    TickType_t xRemaining = xTimeout;

    /* A DONE / ERROR bit left from an aborted transfer must not end this
     * one (K5). The DRDY bit stays. */
    ( void ) ulTaskNotifyValueClear( NULL, IMU_BITS_SPI );

    prvCsLow();

    if( HAL_SPI_TransmitReceive_DMA( &hspi2, ucSpiTx, ucSpiRx, ( uint16_t ) uxLen ) != HAL_OK )
    {
        prvCsHigh();
        return pdFAIL;
    }

    vTaskSetTimeOutState( &xTimeOut );

    for( ; ; )
    {
        /* Clear only the SPI bits on exit: a DRDY that comes meanwhile is
         * kept for the next Imu_WaitDrdy. */
        if( ( xTaskNotifyWait( 0U, IMU_BITS_SPI, &ulBits, xRemaining ) == pdTRUE ) && ( ( ulBits & IMU_BITS_SPI ) != 0U ) )
        {
            break;
        }

        if( xTaskCheckForTimeOut( &xTimeOut, &xRemaining ) != pdFALSE )
        {
            ulBits = 0U;
            ( void ) HAL_SPI_Abort( &hspi2 );       /* app_imu.c:388 */
            break;
        }
    }

    prvCsHigh();

    return ( ( ulBits & IMU_BITS_SPI ) == IMU_BIT_SPI_DONE ) ? pdPASS : pdFAIL;
}

/*---------------------------------------------------------------------------*/
/* prvWaitBits: waits until one of ulBits is in the notification value, at  */
/* most xTimeout (K4). A bit already there counts (it came after the last  */
/* wait). Another bit that wakes the task (a late SPI bit) does not end the */
/* wait: the kernel's own bounded-wait pattern finds the time left.         */
/*---------------------------------------------------------------------------*/
static BaseType_t prvWaitBits( uint32_t ulBits, TickType_t xTimeout )
{
    uint32_t ulValue = 0U;
    TimeOut_t xTimeOut;
    TickType_t xRemaining = xTimeout;

    if( ( ulTaskNotifyValueClear( NULL, ulBits ) & ulBits ) != 0U )
    {
        return pdPASS;
    }

    vTaskSetTimeOutState( &xTimeOut );

    for( ; ; )
    {
        if( ( xTaskNotifyWait( 0U, ulBits, &ulValue, xRemaining ) == pdTRUE ) && ( ( ulValue & ulBits ) != 0U ) )
        {
            return pdPASS;
        }

        if( xTaskCheckForTimeOut( &xTimeOut, &xRemaining ) != pdFALSE )
        {
            return pdFAIL;
        }
    }
}

/*---------------------------------------------------------------------------*/
/* prvUnpack: big-endian i16 pairs from byte 1 (byte 0 answered the address) */
/*---------------------------------------------------------------------------*/
static void prvUnpack( Sample_t * pxOut )
{
    const uint8_t * pucD = &ucSpiRx[ 1 ];
    size_t x;

    for( x = 0U; x < 3U; x++ )
    {
        pxOut->sAccel[ x ] = ( int16_t ) ( ( ( uint16_t ) pucD[ 2U * x ] << 8 ) | pucD[ ( 2U * x ) + 1U ] );
        pxOut->sGyro[ x ] = ( int16_t ) ( ( ( uint16_t ) pucD[ 8U + ( 2U * x ) ] << 8 ) | pucD[ 9U + ( 2U * x ) ] );
    }

    pxOut->sTemp = ( int16_t ) ( ( ( uint16_t ) pucD[ 6 ] << 8 ) | pucD[ 7 ] );
}

/*---------------------------------------------------------------------------*/
/* HAL callbacks (weak in the HAL; app_imu.c:312-352). The pin / instance    */
/* check keeps other EXTI lines and SPIs out (K8). NVIC 6 (EXTI9_5) and 5   */
/* (DMA1 Stream3/4): inside configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.   */
/*---------------------------------------------------------------------------*/
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
    BaseType_t xWoken = pdFALSE;

    if( ( GPIO_Pin == MPU_DRDY_Pin ) && ( xMeasTask != NULL ) )
    {
        ulDrdyCycles = DWT->CYCCNT;                 /* TELEM latency starts here */
        xDrdyTick = xTaskGetTickCountFromISR();     /* K6 */
        ( void ) xTaskNotifyFromISR( xMeasTask, IMU_BIT_DRDY, eSetBits, &xWoken );
        portYIELD_FROM_ISR( xWoken );
    }
}

void HAL_SPI_TxRxCpltCallback( SPI_HandleTypeDef * hspi )
{
    BaseType_t xWoken = pdFALSE;

    if( ( hspi->Instance == SPI2 ) && ( xMeasTask != NULL ) )
    {
        ( void ) xTaskNotifyFromISR( xMeasTask, IMU_BIT_SPI_DONE, eSetBits, &xWoken );
        portYIELD_FROM_ISR( xWoken );
    }
}

void HAL_SPI_ErrorCallback( SPI_HandleTypeDef * hspi )
{
    BaseType_t xWoken = pdFALSE;

    if( ( hspi->Instance == SPI2 ) && ( xMeasTask != NULL ) )
    {
        ( void ) xTaskNotifyFromISR( xMeasTask, IMU_BIT_SPI_ERROR, eSetBits, &xWoken );
        portYIELD_FROM_ISR( xWoken );
    }
}

/*---------------------------------------------------------------------------*/
/* prvWhoAmIOk: 0x70 MPU-6500, 0x71 MPU-9250, 0x73 MPU-9255, 0x74 MPU-6515    */
/* (app_imu.c:75-81). The fitted module answers 0x74 (docs 3.3).             */
/*---------------------------------------------------------------------------*/
static BaseType_t prvWhoAmIOk( uint8_t ucValue )
{
    return ( ( ucValue == 0x70U ) || ( ucValue == 0x71U ) || ( ucValue == 0x73U ) || ( ucValue == 0x74U ) ) ? pdTRUE : pdFALSE;
}

/*---------------------------------------------------------------------------*/
static void prvCsLow( void )
{
    HAL_GPIO_WritePin( MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET );
}

static void prvCsHigh( void )
{
    HAL_GPIO_WritePin( MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET );
}

/*---------------------------------------------------------------------------*/
/* prvSetPrescaler: app_imu.c:151-158. BR can change only with SPE off; the  */
/* HAL turns SPE on again at the next transfer.                              */
/*---------------------------------------------------------------------------*/
static void prvSetPrescaler( uint32_t ulPrescaler )
{
    __HAL_SPI_DISABLE( &hspi2 );
    MODIFY_REG( hspi2.Instance->CR1, SPI_CR1_BR, ulPrescaler );
    hspi2.Init.BaudRatePrescaler = ulPrescaler;
}

/*---------------------------------------------------------------------------*/
/* prvWriteReg / prvReadReg: one 2-byte DMA transfer each (report 092; was  */
/* a polled HAL transfer, app_imu.c:160-186). Imu_Init runs in meas, whose   */
/* notification the SPI callbacks set. A failed transfer is counted and     */
/* then shows in WHO_AM_I or the read-back (K2).                            */
/*---------------------------------------------------------------------------*/
static void prvWriteReg( uint8_t ucReg, uint8_t ucValue )
{
    ucSpiTx[ 0 ] = ucReg;
    ucSpiTx[ 1 ] = ucValue;

    if( prvTransfer( 2U, APP_MS( IMU_REG_TIMEOUT_MS ) ) != pdPASS )
    {
        Stats_Inc( eStatImuSpiTimeout );
    }

    ucSpiTx[ 1 ] = 0x00U;                           /* bursts send zeros */
}

static uint8_t prvReadReg( uint8_t ucReg )
{
    ucSpiTx[ 0 ] = ( uint8_t ) ( ucReg | IMU_READ_FLAG );
    ucSpiRx[ 1 ] = 0x00U;

    if( prvTransfer( 2U, APP_MS( IMU_REG_TIMEOUT_MS ) ) != pdPASS )
    {
        Stats_Inc( eStatImuSpiTimeout );
        return 0x00U;                               /* fails WHO_AM_I / read-back */
    }

    return ucSpiRx[ 1 ];
}
