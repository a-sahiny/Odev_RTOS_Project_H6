/**
  ******************************************************************************
  * @file    app_evt.c
  * @brief   Generic event mechanism (docs 6): event queues, descriptor pools
  *          and the dispatcher loop of ctrlTask and worker.
  ******************************************************************************
  * RTOS objects (künye: report 056), all static:
  *   ctrlQueue      Queue, APP_CTRLQ_LEN x AppEvent_t (8 B).
  *                  Producers, wait 0: critParser (P6, command events), the
  *                  network hook in the IP task (P5, NetUp / NetDown).
  *                  Consumer: ctrlTask only, <= APP_CTRL_MAX_SLEEP_MS.
  *   workQueue      Queue, APP_WORKQ_LEN x AppEvent_t.
  *                  Producer, wait 0: lowParser (P1). Consumer: worker only,
  *                  <= APP_WORK_MAX_SLEEP_MS.
  *   xCritDescPool  fixed_pool (counting semaphore + List_t),
  *                  APP_CRIT_DESC_COUNT blocks. Get: critParser, wait 0.
  *                  Release: ctrlTask (after the handler) or critParser (if
  *                  its AppEvt_Send failed).
  *   xLowDescPool   fixed_pool, APP_LOW_DESC_COUNT blocks. Get: lowParser,
  *                  wait 0. Release: worker or lowParser.
  * Not RTOS objects: the loop timers (AppTimer_t) belong to the loop's task
  * and are read and written by that task only; the loop descriptions
  * (AppEvtLoop_t) are constants of ctrl_task.c / worker_task.c.
  */
#include "app_evt.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"           /* configASSERT, xTaskGetCurrentTaskHandle */
#include "queue.h"

#include "app_config.h"
#include "app_stats.h"      /* Stats_RegisterPool, Stats_GetTaskHandle */
#include "fixed_pool.h"

/* A block is the descriptor header plus the payload capacity of its pool,
 * rounded up so that the next block's ListItem_t is aligned (report 009). */
#define APP_EVT_BLOCK_ALIGN         _Alignof( ListItem_t )
#define APP_EVT_ROUND_UP( x )       ( ( ( ( x ) + APP_EVT_BLOCK_ALIGN - 1U ) / APP_EVT_BLOCK_ALIGN ) * APP_EVT_BLOCK_ALIGN )
#define APP_EVT_CRIT_BLOCK_BYTES    APP_EVT_ROUND_UP( sizeof( AppDesc_t ) + APP_CRIT_DESC_DATA_BYTES )
#define APP_EVT_LOW_BLOCK_BYTES     APP_EVT_ROUND_UP( sizeof( AppDesc_t ) + APP_LOW_DESC_DATA_BYTES )

/* Event queues. NULL = AppEvt_Init not called. */
static StaticQueue_t xCtrlQueueBuf;
static uint8_t ucCtrlQueueStorage[ APP_CTRLQ_LEN * sizeof( AppEvent_t ) ];
static QueueHandle_t xCtrlQueue = NULL;

static StaticQueue_t xWorkQueueBuf;
static uint8_t ucWorkQueueStorage[ APP_WORKQ_LEN * sizeof( AppEvent_t ) ];
static QueueHandle_t xWorkQueue = NULL;

/* Descriptor pools and their blocks. */
static FixedPool_t xCritDescPool;
static FixedPool_t xLowDescPool;
static _Alignas( ListItem_t ) uint8_t ucCritDescBlocks[ APP_CRIT_DESC_COUNT * APP_EVT_CRIT_BLOCK_BYTES ];
static _Alignas( ListItem_t ) uint8_t ucLowDescBlocks[ APP_LOW_DESC_COUNT * APP_EVT_LOW_BLOCK_BYTES ];

/* The pool of each lane, indexed by eAppClass_t (AppDesc_Get / Release). */
static FixedPool_t * const pxDescPools[ eAppClassCount ] = { &xCritDescPool, &xLowDescPool };

static uint8_t ucPoolsInit;     /* 1 after both pools are set up and registered */

/* Where an AppEvt_Send target goes: its queue and the task that reads it
 * (for the "receiver sends to itself" rule, E7). Indexed by eAppTarget_t. */
typedef struct xAPP_EVT_TARGET
{
    QueueHandle_t * pxQueue;
    eAppTaskId_t eReceiver;
} AppEvtTarget_t;

static const AppEvtTarget_t xTargets[ eAppTargetCount ] =
{
    { &xCtrlQueue, eAppTaskCtrl   },    /* eAppTargetCtrl */
    { &xWorkQueue, eAppTaskWorker }     /* eAppTargetWork */
};

static BaseType_t prvInitPool( FixedPool_t * pxPool, uint8_t * pucBlocks, size_t uxBlockSize,
                               UBaseType_t uxBlockCount, UBaseType_t uxIsrReserve, eStatId_t eStatFirst );
static void prvTimerStart( AppTimer_t * pxTimer, TickType_t xTime );
static BaseType_t prvTimerCheck( AppTimer_t * pxTimer );
static void prvCheckTimers( const AppEvtLoop_t * pxLoop );
static TickType_t prvCalculateSleepTime( const AppEvtLoop_t * pxLoop );
static void prvProcessEventsAndTimers( const AppEvtLoop_t * pxLoop );
static void prvDispatch( const AppEvtLoop_t * pxLoop, const AppEvent_t * pxEvent );

_Static_assert( eAppClassCrit == 0, "pxDescPools[ 0 ] is the crit pool" );
_Static_assert( eAppClassLow == 1, "pxDescPools[ 1 ] is the low pool" );
_Static_assert( eAppClassCount == 2, "one pool per lane" );
_Static_assert( ( eAppTargetCtrl == 0 ) && ( eAppTargetWork == 1 ) && ( eAppTargetCount == 2 ), "xTargets order" );
_Static_assert( eAppClassCount <= UINT8_MAX, "the class is kept in ucClass" );
_Static_assert( APP_LOW_DESC_DATA_BYTES <= UINT16_MAX, "usLen is 16 bits" );
_Static_assert( APP_CRIT_DESC_DATA_BYTES <= UINT16_MAX, "usLen is 16 bits" );
_Static_assert( ( APP_CTRLQ_LEN > 0U ) && ( APP_WORKQ_LEN > 0U ), "queues hold at least one event" );

/*---------------------------------------------------------------------------*/
BaseType_t AppEvt_Init( void )
{
    BaseType_t xResult = pdPASS;

    if( xCtrlQueue == NULL )
    {
        /* Static storage: NULL only for bad parameters, which the kernel
         * asserts itself (queue.c:388-394); all fixed here (report 062, B9). */
        xCtrlQueue = xQueueCreateStatic( APP_CTRLQ_LEN, sizeof( AppEvent_t ),
                                         ucCtrlQueueStorage, &xCtrlQueueBuf );
        xWorkQueue = xQueueCreateStatic( APP_WORKQ_LEN, sizeof( AppEvent_t ),
                                         ucWorkQueueStorage, &xWorkQueueBuf );

        #if ( configQUEUE_REGISTRY_SIZE > 0 )
        {
            /* As FreeRTOS_IP.c:1002-1005 does for its event queue. */
            vQueueAddToRegistry( xCtrlQueue, "CtrlQ" );
            vQueueAddToRegistry( xWorkQueue, "WorkQ" );
        }
        #endif
    }

    if( ucPoolsInit == 0U )
    {
        xResult = prvInitPool( &xCritDescPool, ucCritDescBlocks, APP_EVT_CRIT_BLOCK_BYTES,
                               APP_CRIT_DESC_COUNT, APP_CRIT_DESC_ISR_RESERVE, eStatPoolCritEmpty );

        if( xResult == pdPASS )
        {
            xResult = prvInitPool( &xLowDescPool, ucLowDescBlocks, APP_EVT_LOW_BLOCK_BYTES,
                                   APP_LOW_DESC_COUNT, APP_LOW_DESC_ISR_RESERVE, eStatPoolLowEmpty );
        }

        if( xResult == pdPASS )
        {
            ucPoolsInit = 1U;
        }
    }

    return xResult;
}

/*---------------------------------------------------------------------------*/
static BaseType_t prvInitPool( FixedPool_t * pxPool, uint8_t * pucBlocks, size_t uxBlockSize,
                               UBaseType_t uxBlockCount, UBaseType_t uxIsrReserve, eStatId_t eStatFirst )
{
    FixedPoolCfg_t xCfg;
    BaseType_t xResult;

    xCfg.pvBlocks = pucBlocks;
    xCfg.uxBlockSize = uxBlockSize;
    xCfg.uxBlockCount = uxBlockCount;
    xCfg.uxIsrReserve = uxIsrReserve;

    xResult = Pool_Init( pxPool, &xCfg );

    /* Pool counters (empty / invalid / double) through the pool hooks. A
     * group can be registered only once (Stats_RegisterPool asserts). */
    if( ( xResult == pdPASS ) && ( Stats_RegisterPool( pxPool, eStatFirst ) == false ) )
    {
        xResult = pdFAIL;
    }

    return xResult;
}

/*---------------------------------------------------------------------------*/
AppDesc_t * AppDesc_Get( eAppClass_t eClass )
{
    AppDesc_t * pxDesc;

    /* The class is a constant of the caller's lane: a bug otherwise (B9). */
    configASSERT( ( uint32_t ) eClass < ( uint32_t ) eAppClassCount );

    /* Wait 0: a parser never blocks on a pool (docs 4). NULL before
     * AppEvt_Init: Pool_Get checks the semaphore. */
    pxDesc = ( AppDesc_t * ) Pool_Get( pxDescPools[ eClass ], 0U );

    if( pxDesc != NULL )
    {
        /* The only field set here: AppDesc_Release finds the pool by it. */
        pxDesc->ucClass = ( uint8_t ) eClass;
    }

    return pxDesc;
}

/*---------------------------------------------------------------------------*/
void AppDesc_Release( AppDesc_t * pxDesc )
{
    /* NULL: nothing to give back (a NetUp / NetDown event carries none). */
    if( pxDesc != NULL )
    {
        /* ucClass is written only by AppDesc_Get: another value means the
         * block was overwritten, a bug (B9). */
        configASSERT( pxDesc->ucClass < ( uint8_t ) eAppClassCount );

        /* A block of another pool, or one already free, is left untouched
         * and counted by fixed_pool (report 011). */
        ( void ) Pool_Release( pxDescPools[ pxDesc->ucClass ], pxDesc );
    }
}

/*---------------------------------------------------------------------------*/
BaseType_t AppEvt_Send( eAppTarget_t eTarget, const AppEvent_t * pxEvent, TickType_t xTimeout )
{
    TickType_t xUseTimeout = xTimeout;
    QueueHandle_t xQueue;

    /* Internal callers only (parsers, network hook): bad arguments or a
     * missing AppEvt_Init are bugs (B9). */
    configASSERT( ( ( uint32_t ) eTarget < ( uint32_t ) eAppTargetCount ) && ( pxEvent != NULL ) );
    xQueue = *( xTargets[ eTarget ].pxQueue );
    configASSERT( xQueue != NULL );

    /* The receiver cannot block itself while waiting for itself to make room
     * (FreeRTOS_IP.c:1468-1476, xIsCallingFromIPTask). As in the stack, two
     * NULL handles (receiver not created yet, no task running yet) also
     * match; the wait then becomes 0, which is never wrong. */
    if( ( Stats_GetTaskHandle( xTargets[ eTarget ].eReceiver ) == xTaskGetCurrentTaskHandle() ) &&
        ( xUseTimeout > ( TickType_t ) 0U ) )
    {
        xUseTimeout = ( TickType_t ) 0U;
    }

    /* The item is copied: pxEvent may live on the caller's stack. A full
     * queue is not counted here; the caller knows which counter it is (E3). */
    return xQueueSendToBack( xQueue, pxEvent, xUseTimeout );
}

/*---------------------------------------------------------------------------*/
QueueHandle_t AppEvt_GetQueue( eAppTarget_t eTarget )
{
    /* Internal callers only (the two loop owners): a bug otherwise (B9). */
    configASSERT( ( uint32_t ) eTarget < ( uint32_t ) eAppTargetCount );

    return *( xTargets[ eTarget ].pxQueue );
}

/*---------------------------------------------------------------------------*/
/* Timers: IPTimer_t's functions (FreeRTOS_IP_Timers.c)                       */
/*---------------------------------------------------------------------------*/
void AppTimer_Start( AppTimer_t * pxTimer, TickType_t xPeriod )
{
    configASSERT( ( pxTimer != NULL ) && ( xPeriod > ( TickType_t ) 0U ) );   /* B9 */

    /* prvIPTimerReload (:449-453) */
    pxTimer->ulReloadTime = xPeriod;
    prvTimerStart( pxTimer, xPeriod );
}

/*---------------------------------------------------------------------------*/
static void prvTimerStart( AppTimer_t * pxTimer, TickType_t xTime )
{
    /* prvIPTimerStart (:396-411) */
    vTaskSetTimeOutState( &pxTimer->xTimeOut );
    pxTimer->ulRemainingTime = xTime;

    if( xTime == ( TickType_t ) 0U )
    {
        pxTimer->bExpired = 1U;
    }
    else
    {
        pxTimer->bExpired = 0U;
    }

    pxTimer->bActive = 1U;
}

/*---------------------------------------------------------------------------*/
static BaseType_t prvTimerCheck( AppTimer_t * pxTimer )
{
    /* prvIPTimerCheck (:549-578) */
    BaseType_t xReturn;

    if( pxTimer->bActive == 0U )
    {
        /* The timer is not enabled. */
        xReturn = pdFALSE;
    }
    else
    {
        /* Not expired yet: let the kernel compare the start time with the
         * remaining time. It also lowers ulRemainingTime and moves the start
         * to now (tasks.c:5742-5747), which prvCalculateSleepTime reads. */
        if( pxTimer->bExpired == 0U )
        {
            if( xTaskCheckForTimeOut( &( pxTimer->xTimeOut ), &( pxTimer->ulRemainingTime ) ) != pdFALSE )
            {
                pxTimer->bExpired = 1U;
            }
        }

        if( pxTimer->bExpired != 0U )
        {
            /* The next period counts from this check (:572), so a late check
             * delays every later run as well (report 056 E1). */
            prvTimerStart( pxTimer, pxTimer->ulReloadTime );
            xReturn = pdTRUE;
        }
        else
        {
            xReturn = pdFALSE;
        }
    }

    return xReturn;
}

/*---------------------------------------------------------------------------*/
static void prvCheckTimers( const AppEvtLoop_t * pxLoop )
{
    /* vCheckNetworkTimers (:226-389): each due timer runs its work here, in
     * the loop's task. */
    UBaseType_t x;

    for( x = 0U; x < pxLoop->uxTimerCount; x++ )
    {
        if( prvTimerCheck( &( pxLoop->pxTimers[ x ] ) ) != pdFALSE )
        {
            pxLoop->pxTimers[ x ].pfRun();
        }
    }
}

/*---------------------------------------------------------------------------*/
static TickType_t prvCalculateSleepTime( const AppEvtLoop_t * pxLoop )
{
    /* xCalculateSleepTime (:143-224): the loop's longest sleep, shortened to
     * the remaining time of any active timer. */
    TickType_t xMaximumSleepTime = pxLoop->xMaxSleep;
    UBaseType_t x;

    for( x = 0U; x < pxLoop->uxTimerCount; x++ )
    {
        if( pxLoop->pxTimers[ x ].bActive != 0U )
        {
            if( pxLoop->pxTimers[ x ].ulRemainingTime < xMaximumSleepTime )
            {
                xMaximumSleepTime = pxLoop->pxTimers[ x ].ulRemainingTime;
            }
        }
    }

    return xMaximumSleepTime;
}

/*---------------------------------------------------------------------------*/
/* The loop: prvIPTask / prvProcessIPEventsAndTimers (FreeRTOS_IP.c)          */
/*---------------------------------------------------------------------------*/
void AppEvt_RunLoop( const AppEvtLoop_t * pxLoop )
{
    UBaseType_t x;

    /* Checked once: the loop description is a constant of its task. */
    configASSERT( pxLoop != NULL );
    configASSERT( pxLoop->xQueue != NULL );
    configASSERT( pxLoop->pxHandlers != NULL );
    configASSERT( pxLoop->xMaxSleep > ( TickType_t ) 0U );
    configASSERT( ( pxLoop->uxTimerCount == 0U ) || ( pxLoop->pxTimers != NULL ) );

    for( x = 0U; x < pxLoop->uxTimerCount; x++ )
    {
        configASSERT( pxLoop->pxTimers[ x ].pfRun != NULL );
    }

    /* prvIPTask (:235-249) */
    for( ; ; )
    {
        prvProcessEventsAndTimers( pxLoop );
    }
}

/*---------------------------------------------------------------------------*/
static void prvProcessEventsAndTimers( const AppEvtLoop_t * pxLoop )
{
    AppEvent_t xReceivedEvent;
    TickType_t xNextSleep;

    /* No watchdog to feed (ipconfigWATCHDOG_TIMER, :262; E8). */

    /* :266 */
    prvCheckTimers( pxLoop );

    /* :269 */
    xNextSleep = prvCalculateSleepTime( pxLoop );

    /* :274-277. A time out rather than an event: only the timers run. */
    if( xQueueReceive( pxLoop->xQueue, ( void * ) &xReceivedEvent, xNextSleep ) == pdFALSE )
    {
        xReceivedEvent.eEventType = eAppNoEvent;
        xReceivedEvent.pvData = NULL;
    }

    /* :279-292, ipconfigCHECK_IP_QUEUE_SPACE: lowest free space, read by
     * the receiver after taking one event out (E2). */
    if( xReceivedEvent.eEventType != eAppNoEvent )
    {
        Stats_GaugeMin( pxLoop->eGaugeMinFree, ( uint32_t ) uxQueueSpacesAvailable( pxLoop->xQueue ) );
    }

    /* :297-482 */
    prvDispatch( pxLoop, &xReceivedEvent );

    /* No pending work to check (prvIPTask_CheckPendingEvents, :485; E8). */
}

/*---------------------------------------------------------------------------*/
static void prvDispatch( const AppEvtLoop_t * pxLoop, const AppEvent_t * pxEvent )
{
    AppEvtHandler_t pxHandler;

    if( pxEvent->eEventType == eAppNoEvent )
    {
        /* Time out: nothing to hand over. */
    }
    else if( ( uint32_t ) pxEvent->eEventType >= ( uint32_t ) eAppEventCount )
    {
        /* Not an event type at all: only our own code posts events, so this
         * is a bug (B9); pvData cannot be trusted either. */
        configASSERT( 0 );
    }
    else
    {
        pxHandler = pxLoop->pxHandlers[ pxEvent->eEventType ];

        if( pxHandler == NULL )
        {
            /* A known event this loop does not handle (docs 6.2): counted,
             * and its descriptor, if any, goes back to its pool. */
            Stats_Inc( eStatEvtNoHandler );
            AppDesc_Release( ( AppDesc_t * ) pxEvent->pvData );
        }
        else
        {
            /* Counted when it is handed over, as iptraceNETWORK_EVENT_RECEIVED
             * (:295) counts before the switch. */
            Stats_Inc( pxLoop->eStatHandled );

            /* eReleaseBuffer / eFrameConsumed (FreeRTOS_IP.c:1792-1796):
             * the handler says who frees the descriptor. NULL (NetUp /
             * NetDown) is not released: AppDesc_Release ignores it. */
            if( pxHandler( pxEvent->pvData ) == eAppDescRelease )
            {
                AppDesc_Release( ( AppDesc_t * ) pxEvent->pvData );
            }
        }
    }
}
