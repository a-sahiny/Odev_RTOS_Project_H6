/**
  ******************************************************************************
  * @file    meas_task.c
  * @brief   meas (P3): the telemetry source. Every DRDY while NET_READY and
  *          MEAS_EN: one block of IMU points -> CAL offsets -> attitude.c
  *          (Kalman filter: roll / pitch / yaw) -> one TELEM ->
  *          FreeRTOS_sendto. The TELEM rate is the SET_RATE rate (10 / 20 /
  *          50 / 100 / 200 Hz); the filter follows it.
  *          Künye: report 079.
  ******************************************************************************
  * RTOS objects owned:
  *   meas        static task, APP_PRIO_MEAS (3), APP_STACK_MEAS_WORDS. Waits on
  *               the event group (<= APP_MEAS_WAIT_MS), DRDY (<= one sample
  *               period + APP_IMU_DRDY_MARGIN_MS), SPI DMA (<= 5 ms), sendto
  *               (<= 2 ms, SO_SNDTIMEO); 1 s between IMU start-up retries.
  *   xDestBox    static mailbox (queue, length 1, MeasDest_t 12 B). Writer:
  *               ctrlTask (START, xQueueOverwrite). Reader: meas
  *               (xQueuePeek, 0 wait). Report 092: replaces a critical section.
  * Used, not owned: xSysEvents (waits, never writes), its own task
  * notification (imu_spi.c's ISRs), xCalMailbox (the only reader, 0 wait),
  * UDP socket 5002 (the only sender).
  * Shared without an RTOS object:
  *   usRateHz       the SET_RATE rate: ctrlTask writes, meas reads (16-bit
  *                  atomic, K5).
  *   xDrdySlots[]   the DRDY time of each TELEM in flight, by SEQ: meas
  *                  writes, uartTx reads; a short critical section on both
  *                  sides (two words per slot).
  * The overwrite / peek pair is FreeRTOS's mailbox pattern (queue.c
  * xQueueOverwrite): the reader sees one whole item, the old or the new one,
  * never half of each.
  */
#include "meas_task.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

#include "stm32f7xx.h"      /* DWT->CYCCNT (SYNTH source) */

#include "app_config.h"
#include "app_stats.h"
#include "app_types.h"
#include "attitude.h"
#include "imu_spi.h"
#include "pkt_format.h"
#include "raw_link.h"
#include "sys_state.h"
#include "uart_tx.h"        /* UartTx_GetLastTelem */
#include "udp_in.h"
#include "worker_task.h"

#define MEAS_IMU_RETRY_MS       1000U       /* start-up retries (K6) */

_Static_assert( sizeof( "meas" ) <= configMAX_TASK_NAME_LEN, "task name fits" );
_Static_assert( APP_LEN_TELEM == ( ( 9U * 2U ) + 1U + 2U + 2U ), "TELEM = 9 x i16, flags, prev_seq, prev_us" );
_Static_assert( APP_TELEM_FLAGS_OFFSET == ( 9U * 2U ), "flags follow the nine axes" );
_Static_assert( APP_TELEM_PREV_OFFSET == ( APP_TELEM_FLAGS_OFFSET + 1U ), "prev follows flags" );

/* Birim dönüşümleri: ham LSB -> g, rad/s; süzgeç -> TELEM LSB, 0,01 derece. */
#define MEAS_PI                 3.14159265f
#define MEAS_DEG_TO_RAD         ( MEAS_PI / 180.0f )
#define MEAS_RAW_TO_G           ( 1.0f / ( float ) APP_IMU_RAW_LSB_PER_G )
#define MEAS_RAW_TO_RAD         ( ( 10.0f / ( float ) APP_IMU_RAW_LSB_PER_DPS_X10 ) * MEAS_DEG_TO_RAD )
#define MEAS_G_TO_LSB           ( ( float ) APP_IMU_ACCEL_LSB_PER_G )
#define MEAS_RAD_TO_LSB         ( ( ( float ) APP_IMU_GYRO_LSB_PER_DPS_X10 / 10.0f ) / MEAS_DEG_TO_RAD )
#define MEAS_RAD_TO_CDEG        ( 18000.0f / MEAS_PI )

static StaticTask_t xMeasTcb;
static StackType_t xMeasStack[ APP_STACK_MEAS_WORDS ];
static TaskHandle_t xMeasTaskHandle = NULL;     /* NULL = not created yet */

/* Destination, one item of xDestBox (ctrlTask -> meas). ulStarts counts
 * STARTs: a change tells meas that a new measurement began (K2). */
typedef struct xMEAS_DEST
{
    uint32_t ulIpNet;           /* network byte order, as the stack gave it */
    uint16_t usPortNet;
    uint32_t ulStarts;
} MeasDest_t;

static StaticQueue_t xDestBoxBuf;
static uint8_t ucDestBoxStorage[ sizeof( MeasDest_t ) ];
static QueueHandle_t xDestBox = NULL;

/* ctrlTask only (Meas_SetDestination runs there): STARTs given so far. */
static uint32_t ulStartsGiven;

static uint16_t usRateHz = ( uint16_t ) APP_RATE_DEFAULT_HZ;   /* __atomic (K5) */

static const uint16_t usRates[ APP_RATE_COUNT ] = APP_RATE_LIST_HZ;

/* The DRDY time of a TELEM, kept until uartTx has sent its line. */
typedef struct xMEAS_DRDY_SLOT
{
    uint16_t usSeq;
    uint8_t ucUsed;             /* 0: never written                        */
    uint32_t ulCycles;
} MeasDrdySlot_t;

static MeasDrdySlot_t xDrdySlots[ APP_TELEM_DRDY_SLOTS ];

/* meas alone */
typedef struct xMEAS_STATE
{
    uint32_t ulStartsSeen;      /* the START this measurement belongs to   */
    uint16_t usRateInForce;     /* the rate the IMU runs at now            */
    uint16_t usSeq;             /* TELEM SEQ since start-up (K3)           */
#if ( APP_TELEM_TARGET == APP_TELEM_TARGET_MP257 )
    uint16_t usRawSeq;          /* USART6 mesaj sırası                     */
#endif
    TickType_t xDrdyWait;       /* one sample period + margin, in ticks    */
    uint8_t ucRateChanged;      /* 1: next DRDY may still be at the old rate */
    uint8_t ucFailRun;          /* reads in a row that failed (K6)         */
} MeasState_t;

static MeasState_t xMeas;
static QueueHandle_t xCalBox = NULL;

/* meas alone: süzgecin girdisi ve zaman tabanı */
typedef struct xMEAS_IN
{
    ImuBlock_t xBlock;
    AttPoint xPoints[ IMU_BLOCK_MAX ];
    uint32_t ulCalSeen;         /* işlenen son CAL (eStatCalApplied)       */
    uint32_t ulLastTick;
    uint32_t ulLastCycles;
    uint8_t ucHaveLast;         /* 0: no block yet to take dt from         */
} MeasIn_t;

static MeasIn_t xIn;

static void prvMeasTask( void * pvParameters );
static void prvMeasInitialise( void );
static void prvRateInForce( uint16_t usHz );
static void prvApplyNewRate( void );
static BaseType_t prvLoadDestination( MeasDest_t * pxOut );
static BaseType_t prvAcquire( ImuBlock_t * pxOut );
static void prvSendTelemetry( const Sample_t * pxSample, const MeasDest_t * pxDest );
#if ( APP_TELEM_TARGET == APP_TELEM_TARGET_MP257 )
static void prvSendRaw( const Sample_t * pxSample );
#endif
static void prvKeepDrdy( uint16_t usSeq, uint32_t ulCycles );
static void prvAttitudeReset( void );
static void prvFilter( Sample_t * pxOut );
static float prvPointPeriod( void );
static void prvLoadPoints( void );
static int16_t prvToI16( float fValue );

/*---------------------------------------------------------------------------*/
BaseType_t Meas_Init( void )
{
    if( xDestBox == NULL )
    {
        /* Mailbox before the task: ctrlTask and meas use it from their first
         * run. Static storage; the kernel asserts bad parameters. */
        xDestBox = xQueueCreateStatic( 1U, sizeof( MeasDest_t ), ucDestBoxStorage, &xDestBoxBuf );

        #if ( configQUEUE_REGISTRY_SIZE > 0 )
        {
            vQueueAddToRegistry( xDestBox, "MeasDest" );
        }
        #endif
    }

    if( xMeasTaskHandle == NULL )
    {
        /* Static TCB and stack: the kernel asserts NULL buffers itself
         * (report 062, B9). */
        xMeasTaskHandle = xTaskCreateStatic( prvMeasTask, "meas", APP_STACK_MEAS_WORDS, NULL,
                                             APP_PRIO_MEAS, xMeasStack, &xMeasTcb );
        Stats_SetTaskHandle( eAppTaskMeas, xMeasTaskHandle );
    }

    return pdPASS;
}

/*---------------------------------------------------------------------------*/
/* Meas_SetDestination: ctrlTask, START (report 066). The whole item goes    */
/* into the mailbox: meas sees IP, port and START count of one START         */
/* together (K1; report 092).                                                */
/*---------------------------------------------------------------------------*/
void Meas_SetDestination( uint32_t ulIpNet, uint16_t usPortNet )
{
    MeasDest_t xNew;

    configASSERT( xDestBox != NULL );

    ulStartsGiven++;
    xNew.ulIpNet = ulIpNet;
    xNew.usPortNet = usPortNet;
    xNew.ulStarts = ulStartsGiven;

    /* A length-1 queue: overwrite never fails (queue.c xQueueOverwrite). */
    ( void ) xQueueOverwrite( xDestBox, &xNew );
}

/*---------------------------------------------------------------------------*/
BaseType_t Meas_IsRateValid( uint16_t usHz )
{
    BaseType_t xValid = pdFALSE;
    size_t x;

    for( x = 0U; x < APP_RATE_COUNT; x++ )
    {
        if( usRates[ x ] == usHz )
        {
            xValid = pdTRUE;
            break;
        }
    }

    return xValid;
}

/*---------------------------------------------------------------------------*/
void Meas_SetRateHz( uint16_t usHz )
{
    /* cmd.c validated it (E_RANGE otherwise; B8). */
    configASSERT( Meas_IsRateValid( usHz ) == pdTRUE );
    __atomic_store_n( &usRateHz, usHz, __ATOMIC_RELAXED );
}

/*---------------------------------------------------------------------------*/
uint16_t Meas_GetRateHz( void )
{
    return __atomic_load_n( &usRateHz, __ATOMIC_RELAXED );
}

/*---------------------------------------------------------------------------*/
/* Meas_GetTelemDrdy: uartTx, for the TELEM line it has just sent. The slot  */
/* of a SEQ is written APP_TELEM_DRDY_SLOTS TELEMs later again; more than    */
/* that many are never in flight (the TX queue is shorter).                  */
/*---------------------------------------------------------------------------*/
BaseType_t Meas_GetTelemDrdy( uint16_t usSeq, uint32_t * pulCycles )
{
    const MeasDrdySlot_t * pxSlot = &xDrdySlots[ usSeq & ( APP_TELEM_DRDY_SLOTS - 1U ) ];
    BaseType_t xFound = pdFAIL;

    taskENTER_CRITICAL();
    {
        if( ( pxSlot->ucUsed != 0U ) && ( pxSlot->usSeq == usSeq ) )
        {
            *pulCycles = pxSlot->ulCycles;
            xFound = pdPASS;
        }
    }
    taskEXIT_CRITICAL();

    return xFound;
}

/*---------------------------------------------------------------------------*/
static void prvMeasTask( void * pvParameters )
{
    Sample_t xSample;
    MeasDest_t xNow;
#if ( APP_TELEM_TARGET == APP_TELEM_TARGET_MP257 )
    BaseType_t xPcOn;           /* PC yolu açık: NET_READY ve START var */
#endif

    ( void ) pvParameters;

    prvMeasInitialise();

#if ( APP_TELEM_TARGET == APP_TELEM_TARGET_MP257 )
    prvAttitudeReset();         /* START beklenmez: bias ölçümü hemen başlar */
#endif

    for( ; ; )
    {
#if ( APP_TELEM_TARGET == APP_TELEM_TARGET_MP257 )
        /* MP257 hep örnek alır; PC yolu START / STOP'a ve Linux'un yokluğuna bağlı. */
        xPcOn = ( ( SysState_WaitMeasAllowed( 0U ) == pdPASS ) &&
                  ( prvLoadDestination( &xNow ) == pdPASS ) ) ? pdTRUE : pdFALSE;
#else
        /* docs 12: NET_READY and MEAS_EN (ALL). 100 ms rounds: never an
         * unbounded block; STOP / FAULT stop the loop here. */
        if( SysState_WaitMeasAllowed( APP_MS( APP_MEAS_WAIT_MS ) ) != pdPASS )
        {
            continue;
        }

        if( prvLoadDestination( &xNow ) != pdPASS )
        {
            /* MEAS_EN without a START: cannot happen (ctrlTask sets the
             * destination first); wait for the next round. */
            continue;
        }
#endif

        /* SET_RATE: the IMU goes to the new rate before the next sample. */
        if( Meas_GetRateHz() != xMeas.usRateInForce )
        {
            prvApplyNewRate();
        }

#if ( APP_TELEM_TARGET == APP_TELEM_TARGET_MP257 )
        if( ( xPcOn == pdTRUE ) && ( xNow.ulStarts != xMeas.ulStartsSeen ) )
#else
        if( xNow.ulStarts != xMeas.ulStartsSeen )
#endif
        {
            /* A new START: the attitude begins again (K2). A DRDY that came
             * before the START is dropped: the first TELEM is the next fresh
             * sample, so its DRDY -> TX done is not stretched by the time the
             * measurement was off (seen on the board: 34 ms at 10 Hz). */
            xMeas.ulStartsSeen = xNow.ulStarts;
            prvAttitudeReset();
            /* MP257: DRDY sürekli tüketilir, eskisi birikmez; atılırsa örnek kaybolur. */
#if ( ( APP_MEAS_SOURCE == APP_MEAS_SOURCE_IMU ) && ( APP_TELEM_TARGET == APP_TELEM_TARGET_PC ) )
            Imu_DiscardStaleDrdy();
#endif
        }

        if( prvAcquire( &xIn.xBlock ) != pdPASS )
        {
            continue;
        }

        prvFilter( &xSample );

#if ( APP_TELEM_TARGET == APP_TELEM_TARGET_MP257 )
        /* Önce MP257: PC yolunun sendto beklemesi (<= 2 ms) ham süreye girmez. */
        prvSendRaw( &xSample );

        /* Linux okuyorsa TELEM yalnız ondan gider; START yine işlenir. */
        if( ( xPcOn == pdTRUE ) && ( raw_link_linux_on() == 0 ) )
        {
            prvSendTelemetry( &xSample, &xNow );
        }
#else
        prvSendTelemetry( &xSample, &xNow );
#endif
    }
}

/*---------------------------------------------------------------------------*/
/* prvMeasInitialise: the CAL mailbox, then the IMU until it starts (K6).    */
/*---------------------------------------------------------------------------*/
static void prvMeasInitialise( void )
{
    xCalBox = Worker_GetCalMailbox();           /* WorkerTask_Init ran in App_Init */

    att_init();
    prvRateInForce( Meas_GetRateHz() );

#if ( APP_MEAS_SOURCE == APP_MEAS_SOURCE_IMU )
    /* Each failure counts IMU_WHO_AM_I: the health check raises FAULT IMU
     * (report 071), so START is refused until the IMU is back. */
    while( Imu_Init( xTaskGetCurrentTaskHandle(), xMeas.usRateInForce ) != pdPASS )
    {
        vTaskDelay( APP_MS( MEAS_IMU_RETRY_MS ) );
    }

    Imu_DiscardStaleDrdy();
#endif
}

/*---------------------------------------------------------------------------*/
/* prvRateInForce: everything that follows the rate: the DRDY wait, the     */
/* filter's cut-off frequencies and window lengths.                          */
/*---------------------------------------------------------------------------*/
static void prvRateInForce( uint16_t usHz )
{
    xMeas.usRateInForce = usHz;
    xMeas.xDrdyWait = APP_MS( ( 1000U / ( uint32_t ) usHz ) + APP_IMU_DRDY_MARGIN_MS );
    att_set_rate( ( float ) usHz, ( float ) Imu_PointRateHz( usHz ) );
}

/*---------------------------------------------------------------------------*/
/* prvApplyNewRate: SET_RATE gave a new rate. The IMU first; only when it    */
/* runs at the new rate do the filter and the waits follow. The gap around   */
/* the change is not integrated (ucHaveLast = 0). A failed write is counted */
/* by Imu_SetRate (FAULT IMU); the next round tries again.                   */
/*---------------------------------------------------------------------------*/
static void prvApplyNewRate( void )
{
    uint16_t usNew = Meas_GetRateHz();

#if ( APP_MEAS_SOURCE == APP_MEAS_SOURCE_IMU )
    if( Imu_SetRate( usNew ) != pdPASS )
    {
        return;
    }

    Imu_DiscardStaleDrdy();
#endif

    prvRateInForce( usNew );
    xIn.ucHaveLast = 0U;

    /* The IMU takes the new divider only at its next sample at the OLD rate
     * (board, 22.11.2026: 10 -> 100 Hz, three 20 ms DRDY waits ran out
     * before the 100 ms of the old period, FAULT IMU). So the first DRDY
     * after a change gets the longest wait. */
    xMeas.ucRateChanged = 1U;
}

/*---------------------------------------------------------------------------*/
static BaseType_t prvLoadDestination( MeasDest_t * pxOut )
{
    /* A look, not a receive: the destination stays for the next sample. */
    return xQueuePeek( xDestBox, pxOut, 0U );
}

/*---------------------------------------------------------------------------*/
/* prvAcquire: one block at the TELEM rate. IMU: DRDY (<= one period + 10 ms),*/
/* then the DMA read (<= 5 ms); APP_IMU_FAIL_LIMIT failures in a row -> one  */
/* restart of the IMU and IMU_FAIL_RUN counted, which the health check turns */
/* into FAULT IMU (K6). SYNTH: one period, values from a counter (K7).       */
/*---------------------------------------------------------------------------*/
static BaseType_t prvAcquire( ImuBlock_t * pxOut )
{
#if ( APP_MEAS_SOURCE == APP_MEAS_SOURCE_IMU )
    BaseType_t xOk = Imu_WaitDrdy( ( xMeas.ucRateChanged != 0U ) ? APP_MS( APP_IMU_DRDY_TIMEOUT_MAX_MS ) : xMeas.xDrdyWait );

    xMeas.ucRateChanged = 0U;

    if( xOk == pdPASS )
    {
        xOk = Imu_ReadBlock( pxOut, APP_MS( APP_IMU_SPI_TIMEOUT_MS ) );
    }

    if( xOk == pdPASS )
    {
        xMeas.ucFailRun = 0U;
    }
    else
    {
        xMeas.ucFailRun++;

        if( xMeas.ucFailRun >= APP_IMU_FAIL_LIMIT )
        {
            /* Counted for the health check, whatever the rate (a timeout
             * lasts one period + 10 ms); then try once to get the IMU back
             * (a failure counts IMU_WHO_AM_I). */
            xMeas.ucFailRun = 0U;
            Stats_Inc( eStatImuFailRun );

            if( Imu_Init( xTaskGetCurrentTaskHandle(), xMeas.usRateInForce ) == pdPASS )
            {
                Imu_DiscardStaleDrdy();
            }
        }
    }

    return xOk;
#else
    static TickType_t xLastWake = 0U;
    static uint32_t ulCount = 0U;
    size_t x;

    if( xLastWake == 0U )
    {
        xLastWake = xTaskGetTickCount();
    }

    ( void ) xTaskDelayUntil( &xLastWake, APP_MS( 1000U / ( uint32_t ) xMeas.usRateInForce ) );

    for( x = 0U; x < 3U; x++ )
    {
        pxOut->sPoint[ 0 ][ x ] = ( int16_t ) ( uint16_t ) ( ulCount + x );
        pxOut->sPoint[ 0 ][ 3U + x ] = ( int16_t ) ( uint16_t ) ( ( ulCount * 3U ) + x );
    }

    ulCount++;
    pxOut->ulCount = 1U;
    pxOut->ulTick = xLastWake;
    pxOut->ulDrdyCycles = DWT->CYCCNT;

    return pdPASS;
#endif
}

/*---------------------------------------------------------------------------*/
/* prvFilter: son blok -> süzgeç -> TELEM birimleri. Yeni bir CAL            */
/* uygulandıysa (worker'ın sayacı arttı) sıfırlama başlar.                   */
/*---------------------------------------------------------------------------*/
static void prvFilter( Sample_t * pxOut )
{
    AttOutput xOut;
    uint32_t ulCal = Stats_Get( eStatCalApplied );
    size_t x;

    if( ulCal != xIn.ulCalSeen )
    {
        xIn.ulCalSeen = ulCal;
        att_calibrate();
    }

    prvLoadPoints();
    att_update( xIn.xPoints, xIn.xBlock.ulCount, prvPointPeriod(), &xOut );

    for( x = 0U; x < 3U; x++ )
    {
        pxOut->sAccel[ x ] = prvToI16( xOut.acc[ x ] * MEAS_G_TO_LSB );
        pxOut->sGyro[ x ] = prvToI16( xOut.gyro[ x ] * MEAS_RAD_TO_LSB );
        pxOut->sAngle[ x ] = prvToI16( xOut.angle[ x ] * MEAS_RAD_TO_CDEG );
    }

    pxOut->ulDrdyCycles = xIn.xBlock.ulDrdyCycles;
    pxOut->ucFlags = xOut.flags;
}

/*---------------------------------------------------------------------------*/
/* prvLoadPoints: ham noktalar -> g ve rad/s, CAL ofseti (docs 5.2, TELEM    */
/* biriminde) çıkarılmış. Posta kutusu boşsa (hiç CAL yok) ofset yoktur.     */
/*---------------------------------------------------------------------------*/
static void prvLoadPoints( void )
{
    ImuCal_t xCal;
    float fOffset[ 6 ] = { 0.0f };
    uint32_t ul;
    size_t x;

    if( xQueuePeek( xCalBox, &xCal, 0U ) == pdPASS )
    {
        for( x = 0U; x < 3U; x++ )
        {
            fOffset[ x ] = ( float ) xCal.sOffset[ x ] / MEAS_G_TO_LSB;
            fOffset[ 3U + x ] = ( float ) xCal.sOffset[ 3U + x ] / MEAS_RAD_TO_LSB;
        }
    }

    for( ul = 0U; ul < xIn.xBlock.ulCount; ul++ )
    {
        for( x = 0U; x < 3U; x++ )
        {
            xIn.xPoints[ ul ].acc[ x ] = ( ( float ) xIn.xBlock.sPoint[ ul ][ x ] * MEAS_RAW_TO_G ) - fOffset[ x ];
            xIn.xPoints[ ul ].gyro[ x ] = ( ( float ) xIn.xBlock.sPoint[ ul ][ 3U + x ] * MEAS_RAW_TO_RAD ) - fOffset[ 3U + x ];
        }
    }
}

/*---------------------------------------------------------------------------*/
/* prvPointPeriod: iki nokta arası (s), işlemci saatiyle ölçülür: IMU'nun    */
/* kendi saati %1 kayabilir, açı da o kadar yanlış integre edilirdi. 0: ilk  */
/* blok ya da araya boşluk girdi (DWT 19,9 s'de taşar), integre edilmez.     */
/*---------------------------------------------------------------------------*/
static float prvPointPeriod( void )
{
    uint32_t ulTicks = xIn.xBlock.ulTick - xIn.ulLastTick;
    uint32_t ulCycles = xIn.xBlock.ulDrdyCycles - xIn.ulLastCycles;
    float fPeriod = 0.0f;

    if( ( xIn.ucHaveLast != 0U ) && ( ulTicks <= xMeas.xDrdyWait ) )
    {
        fPeriod = ( float ) ulCycles / ( ( float ) SystemCoreClock * ( float ) xIn.xBlock.ulCount );
    }

    xIn.ulLastTick = xIn.xBlock.ulTick;
    xIn.ulLastCycles = xIn.xBlock.ulDrdyCycles;
    xIn.ucHaveLast = 1U;

    return fPeriod;
}

/*---------------------------------------------------------------------------*/
static int16_t prvToI16( float fValue )
{
    float fLimited = fmaxf( -32768.0f, fminf( 32767.0f, fValue ) );

    return ( int16_t ) lroundf( fLimited );
}

/*---------------------------------------------------------------------------*/
/* prvSendTelemetry: TELEM (0x30), 23 B, big-endian: ax ay az gx gy gz i16   */
/* (TELEM LSB, filtered), roll pitch yaw i16 (0.01 deg), flags u8, prev_seq  */
/* u16, prev_us u16. prev: the last TELEM whose line has left the UART and   */
/* its DRDY -> TX done time, measured by uartTx (APP_TELEM_FLAG_PREV). SEQ   */
/* from 0 (K3). The DRDY time is kept for uartTx under this SEQ before the   */
/* send. FreeRTOS_sendto waits at most SO_SNDTIMEO (2 ms) for a network      */
/* buffer and the IP queue (FreeRTOS_Sockets.c:1404-1444); a loss is         */
/* counted (L6).                                                             */
/*---------------------------------------------------------------------------*/
static void prvSendTelemetry( const Sample_t * pxSample, const MeasDest_t * pxDest )
{
    uint8_t ucPayload[ APP_LEN_TELEM ];
    uint8_t ucPacket[ APP_PKT_OVERHEAD_BYTES + APP_LEN_TELEM ];
    PktHeader_t xHdr;
    struct freertos_sockaddr xTo;
    size_t uxLen;
    size_t x;
    int32_t lSent;
    uint32_t ulPrev = UartTx_GetLastTelem();
    uint8_t ucFlags = pxSample->ucFlags;
    Socket_t xSocket = UdpIn_GetTelemSocket();

    /* MEAS_EN implies a START, which needs NET_READY and open sockets. */
    configASSERT( xSocket != NULL );

    for( x = 0U; x < 3U; x++ )
    {
        Pkt_WriteU16( &ucPayload[ 2U * x ], ( uint16_t ) pxSample->sAccel[ x ] );
        Pkt_WriteU16( &ucPayload[ 6U + ( 2U * x ) ], ( uint16_t ) pxSample->sGyro[ x ] );
        Pkt_WriteU16( &ucPayload[ 12U + ( 2U * x ) ], ( uint16_t ) pxSample->sAngle[ x ] );
    }

    if( ulPrev != 0U )
    {
        ucFlags = ( uint8_t ) ( ucFlags | APP_TELEM_FLAG_PREV );
    }

    ucPayload[ APP_TELEM_FLAGS_OFFSET ] = ucFlags;
    Pkt_WriteU16( &ucPayload[ APP_TELEM_PREV_OFFSET ], ( uint16_t ) ( ulPrev >> 16 ) );
    Pkt_WriteU16( &ucPayload[ APP_TELEM_PREV_OFFSET + 2U ], ( uint16_t ) ulPrev );

    xHdr.ucType = APP_TYPE_TELEM;
    xHdr.ucFlags = 0U;
    xHdr.usSeq = xMeas.usSeq;
    xHdr.usReqId = 0U;
    xMeas.usSeq++;

    uxLen = Pkt_Encode( ucPacket, sizeof( ucPacket ), &xHdr, ucPayload, sizeof( ucPayload ) );
    configASSERT( uxLen == sizeof( ucPacket ) );        /* sizes are fixed above */

    ( void ) memset( &xTo, 0, sizeof( xTo ) );
    xTo.sin_len = ( uint8_t ) sizeof( xTo );
    xTo.sin_family = FREERTOS_AF_INET4;
    xTo.sin_port = pxDest->usPortNet;
    xTo.sin_address.ulIP_IPv4 = pxDest->ulIpNet;

    /* Before the send: uartTx may take the line before sendto returns. */
    prvKeepDrdy( xHdr.usSeq, pxSample->ulDrdyCycles );

    Stats_Inc( eStatTelemGen );

    lSent = FreeRTOS_sendto( xSocket, ucPacket, uxLen, 0, &xTo, ( socklen_t ) sizeof( xTo ) );

    Stats_Inc( ( lSent > 0 ) ? eStatTelemSendtoOk : eStatTelemSendtoFail );
}

#if ( APP_TELEM_TARGET == APP_TELEM_TARGET_MP257 )
/*---------------------------------------------------------------------------*/
/* prvSendRaw: prvSendTelemetry ile aynı 23 bayt veri, başlıksız ve CRC'siz. */
/* prev_seq / prev_us USART6 hattının kendi ölçümüdür.                       */
/*---------------------------------------------------------------------------*/
static void prvSendRaw( const Sample_t * pxSample )
{
    uint8_t ucPayload[ APP_LEN_TELEM ];
    uint32_t ulPrev = raw_link_last();
    uint8_t ucFlags = pxSample->ucFlags;
    size_t x;

    for( x = 0U; x < 3U; x++ )
    {
        Pkt_WriteU16( &ucPayload[ 2U * x ], ( uint16_t ) pxSample->sAccel[ x ] );
        Pkt_WriteU16( &ucPayload[ 6U + ( 2U * x ) ], ( uint16_t ) pxSample->sGyro[ x ] );
        Pkt_WriteU16( &ucPayload[ 12U + ( 2U * x ) ], ( uint16_t ) pxSample->sAngle[ x ] );
    }

    if( ulPrev != 0U )
    {
        ucFlags = ( uint8_t ) ( ucFlags | APP_TELEM_FLAG_PREV );
    }

    ucPayload[ APP_TELEM_FLAGS_OFFSET ] = ucFlags;
    Pkt_WriteU16( &ucPayload[ APP_TELEM_PREV_OFFSET ], ( uint16_t ) ( ulPrev >> 16 ) );
    Pkt_WriteU16( &ucPayload[ APP_TELEM_PREV_OFFSET + 2U ], ( uint16_t ) ulPrev );

    raw_link_send( ucPayload, xMeas.usRawSeq, pxSample->ulDrdyCycles );
    xMeas.usRawSeq++;       /* düşen mesaj da sayılır: MP257 boşluğu prev_seq'te görür */
}
#endif

/*---------------------------------------------------------------------------*/
/* prvKeepDrdy: the DRDY time of TELEM usSeq for uartTx (Meas_GetTelemDrdy). */
/*---------------------------------------------------------------------------*/
static void prvKeepDrdy( uint16_t usSeq, uint32_t ulCycles )
{
    MeasDrdySlot_t * const pxSlot = &xDrdySlots[ usSeq & ( APP_TELEM_DRDY_SLOTS - 1U ) ];

    taskENTER_CRITICAL();
    {
        pxSlot->usSeq = usSeq;
        pxSlot->ulCycles = ulCycles;
        pxSlot->ucUsed = 1U;
    }
    taskEXIT_CRITICAL();
}

/*---------------------------------------------------------------------------*/
/* prvAttitudeReset: a new START: yaw, the gyro bias and the time base begin */
/* again (report 087). CAL hizalaması korunur.                               */
/*---------------------------------------------------------------------------*/
static void prvAttitudeReset( void )
{
    att_restart();
    xIn.ucHaveLast = 0U;
}
