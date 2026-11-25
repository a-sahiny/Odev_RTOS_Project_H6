/**
  ******************************************************************************
  * @file    worker_task.c
  * @brief   worker (P2): the low lane's last stop. ENV channel statistics,
  *          BULK pattern check, CAL offsets to meas through a mailbox, the
  *          1 s st / env reports (docs 5.2, 6.4, 17.2). Künye: report 072.
  ******************************************************************************
  * RTOS objects owned:
  *   worker       static task, APP_PRIO_WORKER (2), APP_STACK_WORKER_WORDS.
  *                Waits only on workQueue (<= APP_WORK_MAX_SLEEP_MS; the
  *                report timer wakes it earlier).
  *   xCalMailbox  static queue, length 1, ImuCal_t (12 B). Writer worker
  *                (xQueueOverwrite, never waits); reader meas (0 wait). No
  *                task ever waits on it, so writing it wakes nobody (17.2).
  *   xEnvMailbox  static queue, length 1, WorkerEnvTable_t (136 B). Writer
  *                worker after every ENV packet (xQueueOverwrite); readers
  *                ctrlTask (GET_STATS page 3) and worker (env report), 0 wait.
  *                Report 092: replaces a critical section.
  * Used, not owned: workQueue (the only reader) and the low descriptor pool
  * (app_evt.c), the report pool (report.c, 0 wait).
  * Not RTOS objects: the loop description, the report timer, worker's own
  * working ENV table and the snapshot copy. No critical section here.
  */
#include "worker_task.h"

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "app_config.h"
#include "app_evt.h"
#include "app_stats.h"
#include "app_types.h"
#include "console.h"
#include "pkt_format.h"
#include "report.h"

static StaticTask_t xWorkerTcb;
static StackType_t xWorkerStack[ APP_STACK_WORKER_WORDS ];
static TaskHandle_t xWorkerTask = NULL;         /* NULL = not created yet */

static StaticQueue_t xCalQueueBuffer;
static uint8_t ucCalStorage[ sizeof( ImuCal_t ) ];
static QueueHandle_t xCalMailbox = NULL;

/* One ENV channel since start-up (K4). The sum of i16 values needs more
 * than 32 bits once the count passes 65 536. */
typedef struct xWORKER_ENV_CHANNEL
{
    uint32_t ulCount;
    int16_t sMin;
    int16_t sMax;
    int64_t llSum;
} WorkerEnvChannel_t;

/* The ENV table, one mailbox item (report 092). */
typedef struct xWORKER_ENV_TABLE
{
    WorkerEnvChannel_t xCh[ APP_ENV_MAX_CHANNELS ];
    uint32_t ulSeen;                            /* the largest n so far (K6) */
} WorkerEnvTable_t;

/* worker's working copy: only worker touches it; after each update the
 * whole table goes into the mailbox, readers see one whole table (K5). */
static WorkerEnvTable_t xEnv;

static StaticQueue_t xEnvQueueBuffer;
static uint8_t ucEnvStorage[ sizeof( WorkerEnvTable_t ) ];
static QueueHandle_t xEnvMailbox = NULL;

static eAppDescAction_t prvOnEnv( void * pvData );
static eAppDescAction_t prvOnBulk( void * pvData );
static eAppDescAction_t prvOnCal( void * pvData );
static BaseType_t prvCheckPattern( uint32_t ulSeed, const uint8_t * pucPattern, size_t uxLen );
static void prvWorkerTask( void * pvParameters );
static void prvReportTimer( void );

/* docs 6.2: the events worker handles; the others are counted as
 * EVT_NO_HANDLER by the loop. */
static const AppEvtHandler_t xWorkHandlers[ eAppEventCount ] =
{
    [ eAppEnvEvent ]  = prvOnEnv,
    [ eAppBulkEvent ] = prvOnBulk,
    [ eAppCalEvent ]  = prvOnCal
};

/* The st / env reports (docs 5.2), run by the loop every
 * APP_STATS_REPORT_PERIOD_MS. */
static AppTimer_t xWorkTimers[ 1 ] = { { 0U, 0U, { 0, 0U }, 0U, 0U, prvReportTimer } };

/* The queue handle exists only after AppEvt_Init: written once at the start
 * of the task, constant afterwards (K11, as ctrlTask). */
static AppEvtLoop_t xWorkLoop =
{
    NULL, xWorkHandlers, xWorkTimers, 1U, APP_MS( APP_WORK_MAX_SLEEP_MS ), eStatEvtHandledWork, eGaugeWorkqMinFree
};

_Static_assert( sizeof( "worker" ) <= configMAX_TASK_NAME_LEN, "task name fits" );
_Static_assert( APP_STATS_REPORT_PERIOD_MS <= APP_WORK_MAX_SLEEP_MS, "the loop wakes up for the reports" );
_Static_assert( ( APP_LEN_CAL / 2U ) == ( sizeof( ( ( ImuCal_t * ) 0 )->sOffset ) / sizeof( int16_t ) ), "CAL = 6 x i16" );

/*---------------------------------------------------------------------------*/
BaseType_t WorkerTask_Init( void )
{
    if( xWorkerTask == NULL )
    {
        /* The mailbox first: meas asks for it in its own start-up. Static
         * buffers: the kernel asserts NULL ones itself (report 062, B9). */
        xCalMailbox = xQueueCreateStatic( 1U, sizeof( ImuCal_t ), ucCalStorage, &xCalQueueBuffer );
        vQueueAddToRegistry( xCalMailbox, "CalMbox" );
        xEnvMailbox = xQueueCreateStatic( 1U, sizeof( WorkerEnvTable_t ), ucEnvStorage, &xEnvQueueBuffer );
        vQueueAddToRegistry( xEnvMailbox, "EnvMbox" );

        xWorkerTask = xTaskCreateStatic( prvWorkerTask, "worker", APP_STACK_WORKER_WORDS, NULL,
                                         APP_PRIO_WORKER, xWorkerStack, &xWorkerTcb );

        /* Stack high-water mark; the receiver handle of AppEvt_Send (E7). */
        Stats_SetTaskHandle( eAppTaskWorker, xWorkerTask );
    }

    return pdPASS;
}

/*---------------------------------------------------------------------------*/
QueueHandle_t Worker_GetCalMailbox( void )
{
    configASSERT( xCalMailbox != NULL );            /* WorkerTask_Init in App_Init */
    return xCalMailbox;
}

/*---------------------------------------------------------------------------*/
size_t Worker_GetEnvStats( ReportEnvChannel_t * pxOut, size_t uxMax )
{
    static WorkerEnvTable_t xCopyCtrl;          /* ctrlTask's copy   */
    static WorkerEnvTable_t xCopyWorker;        /* worker's copy     */
    WorkerEnvTable_t * pxCopy;
    size_t uxCount = 0U;
    size_t x;

    configASSERT( ( pxOut != NULL ) && ( xEnvMailbox != NULL ) );

    /* One static copy per caller task (136 B, not on the stacks). worker
     * (P2) may be in the middle of a packet when ctrlTask (P6) asks: the
     * mailbox holds the last whole table (K5, report 092). */
    pxCopy = ( xTaskGetCurrentTaskHandle() == xWorkerTask ) ? &xCopyWorker : &xCopyCtrl;

    if( xQueuePeek( xEnvMailbox, pxCopy, 0U ) == pdPASS )
    {
        uxCount = ( size_t ) pxCopy->ulSeen;
    }

    if( uxCount > uxMax )
    {
        uxCount = uxMax;
    }

    for( x = 0U; x < uxCount; x++ )
    {
        pxOut[ x ].ulCount = pxCopy->xCh[ x ].ulCount;
        pxOut[ x ].sMin = pxCopy->xCh[ x ].sMin;
        pxOut[ x ].sMax = pxCopy->xCh[ x ].sMax;

        /* A channel below the largest n is never empty: the packet that set
         * n filled channels 0..n-1. The mean of i16 values is an i16; C
         * division truncates toward zero (K4). */
        configASSERT( pxCopy->xCh[ x ].ulCount != 0U );
        pxOut[ x ].sMean = ( int16_t ) ( pxCopy->xCh[ x ].llSum / ( int64_t ) pxCopy->xCh[ x ].ulCount );
    }

    return uxCount;
}

/*---------------------------------------------------------------------------*/
static void prvWorkerTask( void * pvParameters )
{
    ( void ) pvParameters;

    xWorkLoop.xQueue = AppEvt_GetQueue( eAppTargetWork );

    /* Reports go out over the UART: nothing to wait for (K11). */
    AppTimer_Start( &xWorkTimers[ 0 ], APP_MS( APP_STATS_REPORT_PERIOD_MS ) );

    /* docs 6.4: the IP task's loop, ours. Never returns. */
    AppEvt_RunLoop( &xWorkLoop );
}

/*---------------------------------------------------------------------------*/
/* prvOnEnv: docs 5.2. n (1..8) + n x i16 (big-endian); value i goes to     */
/* channel i. The parser checked the length only (report 060 P3): n must    */
/* match it, or the channels would shift (K3).                               */
/*---------------------------------------------------------------------------*/
static eAppDescAction_t prvOnEnv( void * pvData )
{
    const AppDesc_t * pxDesc = ( const AppDesc_t * ) pvData;
    int16_t sValue[ APP_ENV_MAX_CHANNELS ];
    size_t uxN;
    size_t x;

    /* lowParser routes ENV with 1 + 2n bytes, n 1..8, only (report 063). */
    configASSERT( ( pxDesc != NULL ) && ( pxDesc->usLen >= 3U ) && ( pxDesc->usLen <= ( 1U + ( 2U * APP_ENV_MAX_CHANNELS ) ) ) );

    uxN = pxDesc->ucData[ 0 ];

    if( ( 1U + ( 2U * uxN ) ) != pxDesc->usLen )
    {
        Stats_Inc( eStatEnvBad );
    }
    else
    {
        for( x = 0U; x < uxN; x++ )
        {
            sValue[ x ] = ( int16_t ) Pkt_ReadU16( &pxDesc->ucData[ 1U + ( 2U * x ) ] );
        }

        /* worker's own table: no other task writes or reads it. */
        for( x = 0U; x < uxN; x++ )
        {
            WorkerEnvChannel_t * pxCh = &xEnv.xCh[ x ];

            if( ( pxCh->ulCount == 0U ) || ( sValue[ x ] < pxCh->sMin ) )
            {
                pxCh->sMin = sValue[ x ];
            }

            if( ( pxCh->ulCount == 0U ) || ( sValue[ x ] > pxCh->sMax ) )
            {
                pxCh->sMax = sValue[ x ];
            }

            pxCh->llSum += sValue[ x ];
            pxCh->ulCount++;
        }

        if( uxN > xEnv.ulSeen )
        {
            xEnv.ulSeen = ( uint32_t ) uxN;
        }

        /* The whole table into the mailbox: readers see this packet's table
         * or the one before, never a mix (report 092). */
        ( void ) xQueueOverwrite( xEnvMailbox, &xEnv );

        Stats_Inc( eStatEnvPkts );
    }

    return eAppDescRelease;
}

/*---------------------------------------------------------------------------*/
/* prvOnBulk: docs 5.2. seed u32 + m pattern bytes, pattern[i] =             */
/* (seed + i) & 0xFF. Counts every packet and its m bytes; a wrong pattern   */
/* is BULK_BAD as well (K7).                                                  */
/*---------------------------------------------------------------------------*/
static eAppDescAction_t prvOnBulk( void * pvData )
{
    const AppDesc_t * pxDesc = ( const AppDesc_t * ) pvData;
    size_t uxPattern;

    /* lowParser routes BULK with 5..200 bytes only (report 063). */
    configASSERT( ( pxDesc != NULL ) &&
                  ( pxDesc->usLen >= ( APP_BULK_SEED_BYTES + APP_BULK_MIN_PATTERN ) ) &&
                  ( pxDesc->usLen <= ( APP_BULK_SEED_BYTES + APP_BULK_MAX_PATTERN ) ) );

    uxPattern = ( size_t ) pxDesc->usLen - APP_BULK_SEED_BYTES;

    Stats_Inc( eStatBulkPkts );
    Stats_Add( eStatBulkBytes, ( uint32_t ) uxPattern );

    if( prvCheckPattern( Pkt_ReadU32( pxDesc->ucData ), &pxDesc->ucData[ APP_BULK_SEED_BYTES ], uxPattern ) == pdFALSE )
    {
        Stats_Inc( eStatBulkBad );
    }

    return eAppDescRelease;
}

/*---------------------------------------------------------------------------*/
/* prvCheckPattern: byte by byte; stops at the first wrong byte.             */
/*---------------------------------------------------------------------------*/
static BaseType_t prvCheckPattern( uint32_t ulSeed, const uint8_t * pucPattern, size_t uxLen )
{
    size_t x;

    for( x = 0U; x < uxLen; x++ )
    {
        if( pucPattern[ x ] != ( uint8_t ) ( ulSeed + ( uint32_t ) x ) )
        {
            return pdFALSE;
        }
    }

    return pdTRUE;
}

/*---------------------------------------------------------------------------*/
/* prvOnCal: docs 5.2. Six i16 offsets (big-endian); all within             */
/* +-APP_CAL_MAX_ABS -> into the mailbox, the last one wins (K8).            */
/*---------------------------------------------------------------------------*/
static eAppDescAction_t prvOnCal( void * pvData )
{
    const AppDesc_t * pxDesc = ( const AppDesc_t * ) pvData;
    ImuCal_t xCal;
    BaseType_t xInRange = pdTRUE;
    size_t x;

    /* lowParser routes CAL with exactly 12 bytes only (report 063). */
    configASSERT( ( pxDesc != NULL ) && ( pxDesc->usLen == APP_LEN_CAL ) );

    for( x = 0U; x < ( APP_LEN_CAL / 2U ); x++ )
    {
        xCal.sOffset[ x ] = ( int16_t ) Pkt_ReadU16( &pxDesc->ucData[ 2U * x ] );

        if( ( xCal.sOffset[ x ] > APP_CAL_MAX_ABS ) || ( xCal.sOffset[ x ] < -APP_CAL_MAX_ABS ) )
        {
            xInRange = pdFALSE;
        }
    }

    if( xInRange == pdTRUE )
    {
        /* Length 1: always succeeds, never waits, wakes nobody (17.2). */
        ( void ) xQueueOverwrite( xCalMailbox, &xCal );
        Stats_Inc( eStatCalApplied );
    }
    else
    {
        Stats_Inc( eStatCalRejected );
    }

    return eAppDescRelease;
}

/*---------------------------------------------------------------------------*/
/* prvReportTimer: docs 5.2 / 18.3, every APP_STATS_REPORT_PERIOD_MS: st    */
/* (resource levels) then env (ENV summaries), every period, also with no   */
/* ENV yet (K10, K17).                                                       */
/*---------------------------------------------------------------------------*/
static void prvReportTimer( void )
{
    /* static: ~0.6 KB + 64 B would not fit the 1.5 KB stack comfortably;
     * worker alone uses them. */
    static StatsSnapshot_t xSnap;
    static ReportEnvChannel_t xEnvOut[ APP_ENV_MAX_CHANNELS ];
    ReportInject_t xInj;
    size_t uxChannels;

    Stats_Snapshot( &xSnap );
    Console_GetInject( &xInj );
    Report_Stats( &xSnap, &xInj );

    uxChannels = Worker_GetEnvStats( xEnvOut, APP_ENV_MAX_CHANNELS );
    Report_Env( ( uint32_t ) xTaskGetTickCount(), xEnvOut, uxChannels );
}
