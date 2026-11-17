/**
  ******************************************************************************
  * @file    meas_task.c
  * @brief   meas (P3): the telemetry source. Every DRDY (100 Hz) while
  *          NET_READY and MEAS_EN: one IMU sample -> CAL offsets -> decimation
  *          to the rate r -> TELEM -> FreeRTOS_sendto (docs 3.3, 15.2).
  *          Every sample also updates roll / pitch / yaw (complementary
  *          filter) for GET_IMU (report 086/087).
  *          Künye: report 079.
  ******************************************************************************
  * RTOS objects owned:
  *   meas        static task, APP_PRIO_MEAS (3), APP_STACK_MEAS_WORDS. Waits on
  *               the event group (<= APP_MEAS_WAIT_MS), DRDY (<= 20 ms), SPI
  *               DMA (<= 5 ms), sendto (<= 2 ms, SO_SNDTIMEO); 1 s between IMU
  *               start-up retries.
  *   xDestBox    static mailbox (queue, length 1, MeasDest_t 12 B). Writer:
  *               ctrlTask (START, xQueueOverwrite). Reader: meas
  *               (xQueuePeek, 0 wait). Report 092: replaces a critical section.
  *   xAttBox     static mailbox (queue, length 1, MeasAttBox_t 32 B). Writer:
  *               meas at every sample (xQueueOverwrite). Reader: ctrlTask
  *               (GET_IMU, xQueuePeek, 0 wait). Report 092.
  * Used, not owned: xSysEvents (waits, never writes), its own task
  * notification (imu_spi.c's ISRs), xCalMailbox (the only reader, 0 wait),
  * UDP socket 5002 (the only sender).
  * Shared without an RTOS object: the rate (16-bit atomic, K5).
  * The overwrite / peek pair is FreeRTOS's mailbox pattern (queue.c
  * xQueueOverwrite): the reader sees one whole item, the old or the new one,
  * never half of each; no critical section in this file.
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

#include "app_config.h"
#include "app_stats.h"
#include "app_types.h"
#include "imu_spi.h"
#include "pkt_format.h"
#include "sys_state.h"
#include "udp_in.h"
#include "worker_task.h"

/* The IMU rate the decimation counts against (docs 3.3). */
#define MEAS_ODR_HZ             ( ( uint16_t ) APP_IMU_ODR_HZ )
#define MEAS_IMU_RETRY_MS       1000U       /* start-up retries (K6) */
#define MEAS_SAMPLE_PERIOD_MS   ( 1000U / APP_IMU_ODR_HZ )

_Static_assert( sizeof( "meas" ) <= configMAX_TASK_NAME_LEN, "task name fits" );
_Static_assert( ( APP_RATE_DEFAULT_HZ >= APP_RATE_MIN_HZ ) && ( APP_RATE_DEFAULT_HZ <= APP_RATE_MAX_HZ ), "default rate in range" );
_Static_assert( APP_RATE_MAX_HZ <= APP_IMU_ODR_HZ, "decimation only" );
_Static_assert( APP_LEN_TELEM == ( 4U + 4U + ( 6U * 2U ) ), "TELEM = idx, tick, 6 x i16 (docs 15.2)" );

/* Attitude (report 087). Angles in radians inside, 0.01 degree outside. */
#define MEAS_PI                 3.14159265f
#define MEAS_DT_S               ( 1.0f / ( float ) APP_IMU_ODR_HZ )                 /* K5 */
#define MEAS_ACC_LSB_PER_G      ( ( float ) APP_IMU_ACCEL_LSB_PER_G )
#define MEAS_GYRO_LSB_PER_RAD   ( ( ( float ) APP_IMU_GYRO_LSB_PER_DPS_X10 / 10.0f ) * ( 180.0f / MEAS_PI ) )
#define MEAS_ALPHA              0.98f       /* gyro weight: tau ~ 0.5 s at 100 Hz   */
#define MEAS_ACC_MIN_G          0.85f       /* |a| outside: linear acceleration,    */
#define MEAS_ACC_MAX_G          1.15f       /* the accelerometer is not used        */
#define MEAS_GIMBAL_LIMIT       ( 85.0f * ( MEAS_PI / 180.0f ) )
#define MEAS_CDEG_PER_RAD       ( 18000.0f / MEAS_PI )
#define MEAS_CDEG_MAX           18000

_Static_assert( APP_ATT_BIAS_SAMPLES > 0U, "bias needs samples" );
_Static_assert( APP_ATT_BIAS_SAMPLES <= 65000U, "bias count is 16 bits; sum of 32768 per sample fits int32" );

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

/* The latest attitude, one item of xAttBox (meas -> ctrlTask), with the
 * START it belongs to. */
typedef struct xMEAS_ATT_BOX
{
    ImuAttitude_t xAtt;
    uint32_t ulStarts;
} MeasAttBox_t;

static StaticQueue_t xDestBoxBuf;
static uint8_t ucDestBoxStorage[ sizeof( MeasDest_t ) ];
static QueueHandle_t xDestBox = NULL;

static StaticQueue_t xAttBoxBuf;
static uint8_t ucAttBoxStorage[ sizeof( MeasAttBox_t ) ];
static QueueHandle_t xAttBox = NULL;

/* ctrlTask only (Meas_SetDestination and Meas_GetAttitude both run there):
 * the number of STARTs given so far. */
static uint32_t ulStartsGiven;

static uint16_t usRateHz = ( uint16_t ) APP_RATE_DEFAULT_HZ;   /* __atomic (K5) */

/* meas alone */
typedef struct xMEAS_STATE
{
    uint32_t ulStartsSeen;      /* the START this measurement belongs to   */
    uint32_t ulIndex;           /* samples since that START (K2)           */
    uint16_t usAcc;             /* decimation accumulator, < MEAS_ODR_HZ   */
    uint16_t usSeq;             /* TELEM SEQ since start-up (K3)           */
    uint8_t ucFailRun;          /* reads in a row that failed (K6)         */
} MeasState_t;

static MeasState_t xMeas;
static QueueHandle_t xCalBox = NULL;

/* meas alone: the filter (report 087) */
typedef struct xMEAS_ATT
{
    float fAngle[ 3 ];          /* roll, pitch, yaw (rad)                  */
    float fBias[ 3 ];           /* gyro bias (LSB), valid once counted     */
    int32_t lBiasSum[ 3 ];
    uint16_t usBiasCount;       /* samples summed, <= APP_ATT_BIAS_SAMPLES */
    uint8_t ucHaveLast;         /* 0: the first sample of this START       */
    uint32_t ulLastTick;
} MeasAtt_t;

static MeasAtt_t xAtt;

static void prvMeasTask( void * pvParameters );
static void prvMeasInitialise( void );
static BaseType_t prvLoadDestination( MeasDest_t * pxOut );
static BaseType_t prvAcquire( Sample_t * pxOut );
static void prvApplyCal( Sample_t * pxSample );
static BaseType_t prvDecimate( void );
static void prvSendTelemetry( const Sample_t * pxSample, const MeasDest_t * pxDest );
static int16_t prvSubSat( int16_t sValue, int16_t sOffset );
static void prvAttitudeReset( void );
static void prvAttitudeUpdate( const Sample_t * pxSample );
static void prvAttitudePublish( const Sample_t * pxSample, uint8_t ucFlags );
static float prvWrapPi( float fAngle );
static int16_t prvCentiDeg( float fAngle );

/*---------------------------------------------------------------------------*/
BaseType_t Meas_Init( void )
{
    if( xDestBox == NULL )
    {
        /* Mailboxes before the task: ctrlTask and meas use them from their
         * first run. Static storage; the kernel asserts bad parameters. */
        xDestBox = xQueueCreateStatic( 1U, sizeof( MeasDest_t ), ucDestBoxStorage, &xDestBoxBuf );
        xAttBox = xQueueCreateStatic( 1U, sizeof( MeasAttBox_t ), ucAttBoxStorage, &xAttBoxBuf );

        #if ( configQUEUE_REGISTRY_SIZE > 0 )
        {
            vQueueAddToRegistry( xDestBox, "MeasDest" );
            vQueueAddToRegistry( xAttBox, "MeasAtt" );
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
void Meas_SetRateHz( uint16_t usHz )
{
    /* cmd.c validated it (E_RANGE otherwise; B8). */
    configASSERT( ( usHz >= APP_RATE_MIN_HZ ) && ( usHz <= APP_RATE_MAX_HZ ) );
    __atomic_store_n( &usRateHz, usHz, __ATOMIC_RELAXED );
}

/*---------------------------------------------------------------------------*/
uint16_t Meas_GetRateHz( void )
{
    return __atomic_load_n( &usRateHz, __ATOMIC_RELAXED );
}

/*---------------------------------------------------------------------------*/
/* Meas_GetAttitude: ctrlTask (GET_IMU). A look into the mailbox with 0      */
/* wait. Valid only for the current START: after a new START the old         */
/* attitude is not answered (K2). ulStartsGiven is ctrlTask's own.           */
/*---------------------------------------------------------------------------*/
BaseType_t Meas_GetAttitude( ImuAttitude_t * pxOut )
{
    MeasAttBox_t xBox;
    BaseType_t xResult = pdFAIL;

    configASSERT( ( pxOut != NULL ) && ( xAttBox != NULL ) );

    if( ( xQueuePeek( xAttBox, &xBox, 0U ) == pdPASS ) && ( xBox.ulStarts == ulStartsGiven ) )
    {
        *pxOut = xBox.xAtt;
        xResult = pdPASS;
    }

    return xResult;
}

/*---------------------------------------------------------------------------*/
static void prvMeasTask( void * pvParameters )
{
    Sample_t xSample;
    MeasDest_t xNow;

    ( void ) pvParameters;

    prvMeasInitialise();

    for( ; ; )
    {
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

        if( xNow.ulStarts != xMeas.ulStartsSeen )
        {
            /* A new START: index and decimation begin again (K2). */
            xMeas.ulStartsSeen = xNow.ulStarts;
            xMeas.ulIndex = 0U;
            xMeas.usAcc = 0U;
            prvAttitudeReset();
        }

        if( prvAcquire( &xSample ) != pdPASS )
        {
            continue;
        }

        xSample.ulIndex = xMeas.ulIndex;
        xMeas.ulIndex++;

        prvApplyCal( &xSample );
        prvAttitudeUpdate( &xSample );

        if( prvDecimate() == pdTRUE )
        {
            prvSendTelemetry( &xSample, &xNow );
        }
    }
}

/*---------------------------------------------------------------------------*/
/* prvMeasInitialise: the CAL mailbox, then the IMU until it starts (K6).    */
/*---------------------------------------------------------------------------*/
static void prvMeasInitialise( void )
{
    xCalBox = Worker_GetCalMailbox();           /* WorkerTask_Init ran in App_Init */

#if ( APP_MEAS_SOURCE == APP_MEAS_SOURCE_IMU )
    /* Each failure counts IMU_WHO_AM_I: the health check raises FAULT IMU
     * (report 071), so START is refused until the IMU is back. */
    while( Imu_Init( xTaskGetCurrentTaskHandle() ) != pdPASS )
    {
        vTaskDelay( APP_MS( MEAS_IMU_RETRY_MS ) );
    }

    Imu_DiscardStaleDrdy();
#endif
}

/*---------------------------------------------------------------------------*/
static BaseType_t prvLoadDestination( MeasDest_t * pxOut )
{
    /* A look, not a receive: the destination stays for the next sample. */
    return xQueuePeek( xDestBox, pxOut, 0U );
}

/*---------------------------------------------------------------------------*/
/* prvAcquire: one sample at the IMU rate. IMU: DRDY (<= 20 ms), then the    */
/* DMA read (<= 5 ms); APP_IMU_FAIL_LIMIT failures in a row -> one restart   */
/* of the IMU (K6). SYNTH: a 10 ms period, values from the index (K7).       */
/*---------------------------------------------------------------------------*/
static BaseType_t prvAcquire( Sample_t * pxOut )
{
#if ( APP_MEAS_SOURCE == APP_MEAS_SOURCE_IMU )
    BaseType_t xOk = Imu_WaitDrdy( APP_MS( APP_IMU_DRDY_TIMEOUT_MS ) );

    if( xOk == pdPASS )
    {
        xOk = Imu_ReadSample( pxOut, APP_MS( APP_IMU_SPI_TIMEOUT_MS ) );
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
            /* The counters already told the health check; try once to get
             * the IMU back (a failure counts IMU_WHO_AM_I). */
            xMeas.ucFailRun = 0U;

            if( Imu_Init( xTaskGetCurrentTaskHandle() ) == pdPASS )
            {
                Imu_DiscardStaleDrdy();
            }
        }
    }

    return xOk;
#else
    static TickType_t xLastWake = 0U;
    size_t x;

    if( xLastWake == 0U )
    {
        xLastWake = xTaskGetTickCount();
    }

    ( void ) xTaskDelayUntil( &xLastWake, APP_MS( MEAS_SAMPLE_PERIOD_MS ) );

    for( x = 0U; x < 3U; x++ )
    {
        pxOut->sAccel[ x ] = ( int16_t ) ( uint16_t ) ( xMeas.ulIndex + x );
        pxOut->sGyro[ x ] = ( int16_t ) ( uint16_t ) ( ( xMeas.ulIndex * 3U ) + x );
    }

    pxOut->sTemp = 0;
    pxOut->ulTick = xLastWake;

    return pdPASS;
#endif
}

/*---------------------------------------------------------------------------*/
/* prvApplyCal: docs 5.2 / 17.2. A look into the mailbox with 0 wait; empty  */
/* (no CAL yet) = no offsets. Offsets are subtracted, saturating (K4).       */
/*---------------------------------------------------------------------------*/
static void prvApplyCal( Sample_t * pxSample )
{
    ImuCal_t xCal;
    size_t x;

    if( xQueuePeek( xCalBox, &xCal, 0U ) == pdPASS )
    {
        for( x = 0U; x < 3U; x++ )
        {
            pxSample->sAccel[ x ] = prvSubSat( pxSample->sAccel[ x ], xCal.sOffset[ x ] );
            pxSample->sGyro[ x ] = prvSubSat( pxSample->sGyro[ x ], xCal.sOffset[ 3U + x ] );
        }
    }
}

/*---------------------------------------------------------------------------*/
static int16_t prvSubSat( int16_t sValue, int16_t sOffset )
{
    int32_t lDiff = ( int32_t ) sValue - ( int32_t ) sOffset;

    if( lDiff > INT16_MAX )
    {
        lDiff = INT16_MAX;
    }
    else if( lDiff < INT16_MIN )
    {
        lDiff = INT16_MIN;
    }
    else
    {
        /* in range */
    }

    return ( int16_t ) lDiff;
}

/*---------------------------------------------------------------------------*/
/* prvDecimate: docs 3.3. acc += r; acc >= 100 -> acc -= 100 and send. The   */
/* mean rate is exactly r; a new r takes effect at this sample.             */
/*---------------------------------------------------------------------------*/
static BaseType_t prvDecimate( void )
{
    BaseType_t xSend = pdFALSE;

    xMeas.usAcc = ( uint16_t ) ( xMeas.usAcc + Meas_GetRateHz() );

    if( xMeas.usAcc >= MEAS_ODR_HZ )
    {
        xMeas.usAcc = ( uint16_t ) ( xMeas.usAcc - MEAS_ODR_HZ );
        xSend = pdTRUE;
    }

    return xSend;
}

/*---------------------------------------------------------------------------*/
/* prvSendTelemetry: docs 15.2 TELEM (0x30), 20 B: sample_idx u32,           */
/* sample_tick u32, ax ay az gx gy gz i16, big-endian. SEQ from 0 (K3).      */
/* FreeRTOS_sendto waits at most SO_SNDTIMEO (2 ms) for a network buffer and */
/* the IP queue (FreeRTOS_Sockets.c:1404-1444); a loss is counted (L6).      */
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
    Socket_t xSocket = UdpIn_GetTelemSocket();

    /* MEAS_EN implies a START, which needs NET_READY and open sockets. */
    configASSERT( xSocket != NULL );

    Pkt_WriteU32( &ucPayload[ 0 ], pxSample->ulIndex );
    Pkt_WriteU32( &ucPayload[ 4 ], pxSample->ulTick );

    for( x = 0U; x < 3U; x++ )
    {
        Pkt_WriteU16( &ucPayload[ 8U + ( 2U * x ) ], ( uint16_t ) pxSample->sAccel[ x ] );
        Pkt_WriteU16( &ucPayload[ 14U + ( 2U * x ) ], ( uint16_t ) pxSample->sGyro[ x ] );
    }

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

    Stats_Inc( eStatTelemGen );

    lSent = FreeRTOS_sendto( xSocket, ucPacket, uxLen, 0, &xTo, ( socklen_t ) sizeof( xTo ) );

    Stats_Inc( ( lSent > 0 ) ? eStatTelemSendtoOk : eStatTelemSendtoFail );
}

/*---------------------------------------------------------------------------*/
/* prvAttitudeReset: a new START: the angles, the gyro bias and the time     */
/* base begin again (report 087).                                            */
/*---------------------------------------------------------------------------*/
static void prvAttitudeReset( void )
{
    ( void ) memset( &xAtt, 0, sizeof( xAtt ) );
}

/*---------------------------------------------------------------------------*/
/* prvAttitudeUpdate: one sample (CAL applied) -> roll, pitch, yaw.          */
/*  - First APP_ATT_BIAS_SAMPLES samples after START: the gyro bias is       */
/*    their mean; roll / pitch from the accelerometer alone, yaw 0.          */
/*  - Then: the gyro rates (bias removed) move the angles through the Euler  */
/*    kinematics; roll / pitch are pulled towards the accelerometer angles   */
/*    (complementary filter, MEAS_ALPHA) while |a| is about 1 g. Yaw comes   */
/*    from the gyro alone (no magnetometer): it drifts.                      */
/*  dt is the fixed sample period; a sample whose tick is not 1..20 ms after */
/*  the last one is not integrated (K5).                                     */
/*---------------------------------------------------------------------------*/
static void prvAttitudeUpdate( const Sample_t * pxSample )
{
    float fAcc[ 3 ];
    float fRate[ 3 ];
    float fAccRoll;
    float fAccPitch;
    float fNorm;
    float fSinR;
    float fCosR;
    float fCosP;
    float fBodyQr;
    uint32_t ulDt;
    uint8_t ucFlags = 0U;
    BaseType_t xAccOk;
    BaseType_t xDtOk;
    size_t x;

    for( x = 0U; x < 3U; x++ )
    {
        fAcc[ x ] = ( float ) pxSample->sAccel[ x ] / MEAS_ACC_LSB_PER_G;
    }

    fNorm = sqrtf( ( fAcc[ 0 ] * fAcc[ 0 ] ) + ( fAcc[ 1 ] * fAcc[ 1 ] ) + ( fAcc[ 2 ] * fAcc[ 2 ] ) );
    xAccOk = ( ( fNorm >= MEAS_ACC_MIN_G ) && ( fNorm <= MEAS_ACC_MAX_G ) ) ? pdTRUE : pdFALSE;
    fAccRoll = atan2f( fAcc[ 1 ], fAcc[ 2 ] );
    fAccPitch = atan2f( -fAcc[ 0 ], sqrtf( ( fAcc[ 1 ] * fAcc[ 1 ] ) + ( fAcc[ 2 ] * fAcc[ 2 ] ) ) );

    ulDt = pxSample->ulTick - xAtt.ulLastTick;
    xDtOk = ( ( xAtt.ucHaveLast != 0U ) && ( ulDt != 0U ) && ( ulDt <= APP_MS( APP_IMU_DRDY_TIMEOUT_MS ) ) ) ? pdTRUE : pdFALSE;

    if( xAtt.usBiasCount < APP_ATT_BIAS_SAMPLES )
    {
        /* Bias phase: the board is expected to stay still (known limit). */
        for( x = 0U; x < 3U; x++ )
        {
            xAtt.lBiasSum[ x ] += ( int32_t ) pxSample->sGyro[ x ];
        }

        xAtt.usBiasCount++;

        if( xAtt.usBiasCount == APP_ATT_BIAS_SAMPLES )
        {
            for( x = 0U; x < 3U; x++ )
            {
                xAtt.fBias[ x ] = ( float ) xAtt.lBiasSum[ x ] / ( float ) APP_ATT_BIAS_SAMPLES;
            }
        }

        if( ( xAccOk == pdTRUE ) || ( xAtt.ucHaveLast == 0U ) )
        {
            /* The first sample takes the accelerometer angles in any case:
             * there is nothing better. */
            xAtt.fAngle[ 0 ] = fAccRoll;
            xAtt.fAngle[ 1 ] = fAccPitch;
            ucFlags = ( xAccOk == pdTRUE ) ? ( uint8_t ) APP_ATT_FLAG_ACC_USED : 0U;
        }

        xAtt.fAngle[ 2 ] = 0.0f;
    }
    else
    {
        if( xDtOk == pdTRUE )
        {
            for( x = 0U; x < 3U; x++ )
            {
                fRate[ x ] = ( ( float ) pxSample->sGyro[ x ] - xAtt.fBias[ x ] ) / MEAS_GYRO_LSB_PER_RAD;
            }

            /* Euler kinematics: body rates p q r -> roll, pitch, yaw rates. */
            fSinR = sinf( xAtt.fAngle[ 0 ] );
            fCosR = cosf( xAtt.fAngle[ 0 ] );
            fCosP = cosf( xAtt.fAngle[ 1 ] );
            fBodyQr = ( fRate[ 1 ] * fSinR ) + ( fRate[ 2 ] * fCosR );

            if( fabsf( xAtt.fAngle[ 1 ] ) < MEAS_GIMBAL_LIMIT )
            {
                xAtt.fAngle[ 0 ] += ( fRate[ 0 ] + ( fBodyQr * ( sinf( xAtt.fAngle[ 1 ] ) / fCosP ) ) ) * MEAS_DT_S;
                xAtt.fAngle[ 2 ] += ( fBodyQr / fCosP ) * MEAS_DT_S;
            }
            else
            {
                /* Near +-90 degree pitch 1/cos(pitch) explodes (gimbal
                 * lock): roll takes p alone, yaw stands (known limit). */
                xAtt.fAngle[ 0 ] += fRate[ 0 ] * MEAS_DT_S;
            }

            xAtt.fAngle[ 1 ] += ( ( fRate[ 1 ] * fCosR ) - ( fRate[ 2 ] * fSinR ) ) * MEAS_DT_S;
        }

        if( xAccOk == pdTRUE )
        {
            /* Towards the accelerometer, roll by the shorter way round. */
            xAtt.fAngle[ 0 ] += ( 1.0f - MEAS_ALPHA ) * prvWrapPi( fAccRoll - xAtt.fAngle[ 0 ] );
            xAtt.fAngle[ 1 ] += ( 1.0f - MEAS_ALPHA ) * ( fAccPitch - xAtt.fAngle[ 1 ] );
            ucFlags = ( uint8_t ) APP_ATT_FLAG_ACC_USED;
        }

        ucFlags = ( uint8_t ) ( ucFlags | APP_ATT_FLAG_BIAS_READY );
    }

    xAtt.fAngle[ 0 ] = prvWrapPi( xAtt.fAngle[ 0 ] );
    xAtt.fAngle[ 1 ] = fmaxf( -0.5f * MEAS_PI, fminf( 0.5f * MEAS_PI, xAtt.fAngle[ 1 ] ) );
    xAtt.fAngle[ 2 ] = prvWrapPi( xAtt.fAngle[ 2 ] );
    xAtt.ulLastTick = pxSample->ulTick;
    xAtt.ucHaveLast = 1U;

    prvAttitudePublish( pxSample, ucFlags );
}

/*---------------------------------------------------------------------------*/
/* prvAttitudePublish: the reply view into the mailbox (report 092): the    */
/* kernel copies the whole item, ctrlTask (P6) never sees half of it (K2).   */
/*---------------------------------------------------------------------------*/
static void prvAttitudePublish( const Sample_t * pxSample, uint8_t ucFlags )
{
    MeasAttBox_t xNew;
    size_t x;

    xNew.xAtt.ulIndex = pxSample->ulIndex;
    xNew.xAtt.ulTick = pxSample->ulTick;

    for( x = 0U; x < 3U; x++ )
    {
        xNew.xAtt.sAccel[ x ] = pxSample->sAccel[ x ];
        xNew.xAtt.sGyro[ x ] = pxSample->sGyro[ x ];
        xNew.xAtt.sAngle[ x ] = prvCentiDeg( xAtt.fAngle[ x ] );
    }

    xNew.xAtt.ucFlags = ucFlags;
    xNew.ulStarts = xMeas.ulStartsSeen;

    ( void ) xQueueOverwrite( xAttBox, &xNew );
}

/*---------------------------------------------------------------------------*/
/* prvWrapPi: into [-pi, pi). One step is enough: an update moves an angle   */
/* by far less than 2 pi.                                                    */
/*---------------------------------------------------------------------------*/
static float prvWrapPi( float fAngle )
{
    float fOut = fAngle;

    if( fOut >= MEAS_PI )
    {
        fOut -= 2.0f * MEAS_PI;
    }
    else if( fOut < -MEAS_PI )
    {
        fOut += 2.0f * MEAS_PI;
    }
    else
    {
        /* in range */
    }

    return fOut;
}

/*---------------------------------------------------------------------------*/
/* prvCentiDeg: radians -> 0.01 degree, rounded, limited to +-180.00.        */
/*---------------------------------------------------------------------------*/
static int16_t prvCentiDeg( float fAngle )
{
    float fValue = fAngle * MEAS_CDEG_PER_RAD;
    int32_t lValue;

    fValue += ( fValue >= 0.0f ) ? 0.5f : -0.5f;
    lValue = ( int32_t ) fValue;

    if( lValue > MEAS_CDEG_MAX )
    {
        lValue = MEAS_CDEG_MAX;
    }
    else if( lValue < -MEAS_CDEG_MAX )
    {
        lValue = -MEAS_CDEG_MAX;
    }
    else
    {
        /* in range */
    }

    return ( int16_t ) lValue;
}
