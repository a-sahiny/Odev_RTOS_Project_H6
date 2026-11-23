/**
  ******************************************************************************
  * @file    app_stats.c
  * @brief   Counters, latency histograms, gauges and snapshots (docs 8.1, 18).
  ******************************************************************************
  * No RTOS objects. Shared memory written by many tasks and ISRs:
  *   ulCounters[]  writers: every layer (tasks, UART ISRs, IP task, trace
  *                 hooks); readers: Stats_Get, Stats_Snapshot.
  *                 Lock-free atomic add (LDREX/STREX on Cortex-M7).
  *   ulGauges[]    writers: UART RX ISR, IP task, parsers, report writers;
  *                 readers: Stats_GaugeGet, Stats_Snapshot.
  *                 Lock-free compare-and-swap (LDREX/STREX).
  *   xLat[]        ONE writer task per latency (critParser, ctrlTask,
  *                 uartTx); readers: Stats_LatencyGet, Stats_Snapshot. Two
  *                 copies and a generation counter (report 092): the writer
  *                 fills the copy no reader uses and publishes it with one
  *                 32-bit store; a reader keeps its copy only if the
  *                 generation did not move meanwhile. No lock: a reader of
  *                 higher priority that preempts the writer still finds the
  *                 published copy untouched.
  *   xTaskHandles[], xPools[]  written only in App_Init, before the
  *                 scheduler; read-only afterwards, so the pool hooks may
  *                 read them from ISRs without a lock.
  * Stats_Snapshot copies ulCounters[] and ulGauges[] and checks the copy
  * against the live values (report 092): counters only grow and every gauge
  * moves one way only, so an unchanged second look proves that the copy
  * holds the values of one instant. No critical section in this file.
  */
#include "app_stats.h"

#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"       /* configASSERT uses taskDISABLE_INTERRUPTS */

#include "FreeRTOS_IP.h"                /* uxGetMinimumIPQueueSpace, IP task handle */
#include "NetworkBufferManagement.h"    /* network buffer levels                    */
#include "fixed_pool.h"                 /* Pool_GetFree, Pool_GetMinFree            */

static uint32_t ulCounters[ eStatCount ];
static uint32_t ulGauges[ eGaugeCount ];
/* One latency: two copies, readers use xBuf[ ulGen & 1 ] (report 092). */
typedef struct xLAT_PUB
{
    LatHist_t xBuf[ 2 ];
    uint32_t ulGen;                 /* written by the one writer only (atomic) */
} LatPub_t;

static LatPub_t xLat[ eLatCount ];

/* Copy-and-check rounds of Stats_Snapshot before it keeps the last copy. */
#define STATS_SNAPSHOT_TRIES    8U

typedef struct xSTATS_POOL_ENTRY
{
    const struct xFIXED_POOL * pxPool;      /* NULL = free slot               */
    eStatId_t eFirst;                       /* first id of its counter group  */
} StatsPoolEntry_t;

static struct tskTaskControlBlock * xTaskHandles[ eAppTaskCount ];
static StatsPoolEntry_t xPools[ STATS_MAX_POOLS ];

/* Group layout the pool events rely on: counter = eFirst + event. */
_Static_assert( ( eStatPoolCritInvalid - eStatPoolCritEmpty ) == eStatPoolEvtInvalid, "pool group layout" );
_Static_assert( ( eStatPoolCritDouble - eStatPoolCritEmpty ) == eStatPoolEvtDouble, "pool group layout" );
_Static_assert( ( eStatPoolLowEmpty - eStatPoolCritEmpty ) == eStatPoolEvtCount, "pool group layout" );
_Static_assert( ( eStatPoolReportEmpty - eStatPoolLowEmpty ) == eStatPoolEvtCount, "pool group layout" );
_Static_assert( ( eStatPoolUnregistered - eStatPoolReportEmpty ) == eStatPoolEvtCount, "pool group layout" );

static void prvGaugeUpdate( eGaugeId_t eId, uint32_t ulValue, bool bKeepMax );
static void prvSampleResources( StatsSnapshot_t * pxOut );
static uint32_t prvStackHwm( TaskHandle_t xTask );

/*---------------------------------------------------------------------------*/
/* Stats_Init                                                                */
/*---------------------------------------------------------------------------*/
void Stats_Init( void )
{
    /* Runs before the scheduler and before any interrupt that counts is
     * enabled, so a plain clear is enough. */
    uint32_t x;

    ( void ) memset( ulCounters, 0, sizeof( ulCounters ) );
    ( void ) memset( xLat, 0, sizeof( xLat ) );
    ( void ) memset( xTaskHandles, 0, sizeof( xTaskHandles ) );
    ( void ) memset( xPools, 0, sizeof( xPools ) );

    /* High-water gauges start at 0, low-water gauges at the largest value,
     * so the first real value always replaces the start value. */
    for( x = 0U; x < ( uint32_t ) eGaugeCount; x++ )
    {
        ulGauges[ x ] = ( x < ( uint32_t ) eGaugeFirstMin ) ? 0U : UINT32_MAX;
    }
}

/*---------------------------------------------------------------------------*/
/* Stats_Inc / Stats_Add                                                     */
/*---------------------------------------------------------------------------*/
void Stats_Inc( eStatId_t eId )
{
    Stats_Add( eId, 1U );
}

void Stats_Add( eStatId_t eId, uint32_t ulValue )
{
    /* Ids are constants of the code: a bad one is a bug, and configASSERT
     * never returns in this project (report 062, B9). */
    configASSERT( ( uint32_t ) eId < ( uint32_t ) eStatCount );

    /* Read-modify-write as one atomic operation: LDREX / ADD / STREX,
     * retried if anyone else wrote the word in between. A plain "+=" could
     * lose an increment made by an interrupt between the read and the
     * write. Relaxed order: a counter orders nothing else. */
    ( void ) __atomic_fetch_add( &ulCounters[ eId ], ulValue, __ATOMIC_RELAXED );
}

/*---------------------------------------------------------------------------*/
/* Stats_Get                                                                 */
/*---------------------------------------------------------------------------*/
uint32_t Stats_Get( eStatId_t eId )
{
    uint32_t ulValue = 0U;

    if( ( uint32_t ) eId < ( uint32_t ) eStatCount )
    {
        /* An aligned 32-bit load: never half old, half new. */
        ulValue = __atomic_load_n( &ulCounters[ eId ], __ATOMIC_RELAXED );
    }

    return ulValue;
}

/*---------------------------------------------------------------------------*/
/* Stats_GaugeMax / Stats_GaugeMin                                           */
/*---------------------------------------------------------------------------*/
void Stats_GaugeMax( eGaugeId_t eId, uint32_t ulValue )
{
    configASSERT( ( uint32_t ) eId < ( uint32_t ) eGaugeFirstMin );     /* a max gauge (B9) */
    prvGaugeUpdate( eId, ulValue, true );
}

void Stats_GaugeMin( eGaugeId_t eId, uint32_t ulValue )
{
    configASSERT( ( ( uint32_t ) eId >= ( uint32_t ) eGaugeFirstMin ) &&
                  ( ( uint32_t ) eId < ( uint32_t ) eGaugeCount ) );   /* a min gauge (B9) */
    prvGaugeUpdate( eId, ulValue, false );
}

/*---------------------------------------------------------------------------*/
/* prvGaugeUpdate: store ulValue only if it is a new extreme                 */
/*---------------------------------------------------------------------------*/
static void prvGaugeUpdate( eGaugeId_t eId, uint32_t ulValue, bool bKeepMax )
{
    uint32_t ulOld = __atomic_load_n( &ulGauges[ eId ], __ATOMIC_RELAXED );

    /* Compare-and-swap loop: write ulValue only if the gauge still holds
     * ulOld. If someone (a task or an interrupt) changed it in between, the
     * swap fails, ulOld is reloaded with the new value and the comparison is
     * made again. The loop ends as soon as ulValue is no longer a new
     * extreme, so a value that is not an extreme costs one load only. */
    while( bKeepMax ? ( ulValue > ulOld ) : ( ulValue < ulOld ) )
    {
        if( __atomic_compare_exchange_n( &ulGauges[ eId ], &ulOld, ulValue,
                                         true, __ATOMIC_RELAXED, __ATOMIC_RELAXED ) )
        {
            break;
        }
    }
}

/*---------------------------------------------------------------------------*/
/* Stats_GaugeGet                                                            */
/*---------------------------------------------------------------------------*/
uint32_t Stats_GaugeGet( eGaugeId_t eId )
{
    uint32_t ulValue = 0U;

    if( ( uint32_t ) eId < ( uint32_t ) eGaugeCount )
    {
        ulValue = __atomic_load_n( &ulGauges[ eId ], __ATOMIC_RELAXED );
    }

    return ulValue;
}

/*---------------------------------------------------------------------------*/
/* Stats_LatencyAdd                                                          */
/*---------------------------------------------------------------------------*/
void Stats_LatencyAdd( eLatId_t eId, uint32_t ulCycles )
{
    LatHist_t * pxHist;
    uint32_t ulCyclesPerUs;
    uint32_t ulUs;
    uint32_t ulGen;

    configASSERT( ( uint32_t ) eId < ( uint32_t ) eLatCount );         /* B9 */

    /* configCPU_CLOCK_HZ = SystemCoreClock (216 MHz): 216 cycles per us. */
    ulCyclesPerUs = ( uint32_t ) ( configCPU_CLOCK_HZ / 1000000UL );

    if( ulCyclesPerUs == 0U )
    {
        ulCyclesPerUs = 1U;
    }

    ulUs = ulCycles / ulCyclesPerUs;

    /* Three fields form one sample: count, sum, max (report 062, B1). The
     * writer (one task per latency) builds the next copy in the buffer no
     * reader uses, then publishes it with one store (report 092). */
    ulGen = __atomic_load_n( &( xLat[ eId ].ulGen ), __ATOMIC_RELAXED );
    pxHist = &( xLat[ eId ].xBuf[ ( ulGen + 1U ) & 1U ] );
    *pxHist = xLat[ eId ].xBuf[ ulGen & 1U ];

    pxHist->ulCount++;
    pxHist->ullSumUs += ulUs;

    if( ulUs > pxHist->ulMaxUs )
    {
        pxHist->ulMaxUs = ulUs;
    }

    __atomic_store_n( &( xLat[ eId ].ulGen ), ulGen + 1U, __ATOMIC_RELEASE );
}

/*---------------------------------------------------------------------------*/
/* Stats_LatencyGet                                                          */
/*---------------------------------------------------------------------------*/
void Stats_LatencyGet( eLatId_t eId, LatHist_t * pxOut )
{
    if( pxOut != NULL )
    {
        if( ( uint32_t ) eId < ( uint32_t ) eLatCount )
        {
            uint32_t ulGen;
            uint32_t ulAfter;

            /* The published copy is xBuf[ gen & 1 ]. The writer only writes
             * the other one, until it publishes gen + 1; after that its next
             * sample may rewrite ours. So the copy counts only if the
             * generation is the same before and after. A reader of higher
             * priority never sees it move (the writer cannot run); a lower
             * one may try again. */
            do
            {
                ulGen = __atomic_load_n( &( xLat[ eId ].ulGen ), __ATOMIC_ACQUIRE );
                *pxOut = xLat[ eId ].xBuf[ ulGen & 1U ];
                __atomic_thread_fence( __ATOMIC_ACQUIRE );
                ulAfter = __atomic_load_n( &( xLat[ eId ].ulGen ), __ATOMIC_RELAXED );
            } while( ulAfter != ulGen );
        }
        else
        {
            ( void ) memset( pxOut, 0, sizeof( *pxOut ) );
        }
    }
}

/*---------------------------------------------------------------------------*/
/* Stats_SetTaskHandle / Stats_GetTaskHandle                                 */
/*---------------------------------------------------------------------------*/
void Stats_SetTaskHandle( eAppTaskId_t eTask, struct tskTaskControlBlock * xHandle )
{
    configASSERT( ( uint32_t ) eTask < ( uint32_t ) eAppTaskCount );    /* B9 */
    xTaskHandles[ eTask ] = xHandle;
}

struct tskTaskControlBlock * Stats_GetTaskHandle( eAppTaskId_t eTask )
{
    struct tskTaskControlBlock * xHandle = NULL;

    if( ( uint32_t ) eTask < ( uint32_t ) eAppTaskCount )
    {
        xHandle = xTaskHandles[ eTask ];
    }

    return xHandle;
}

/*---------------------------------------------------------------------------*/
/* Stats_RegisterPool                                                        */
/*---------------------------------------------------------------------------*/
bool Stats_RegisterPool( const struct xFIXED_POOL * pxPool, eStatId_t eFirst )
{
    bool bOk = false;
    uint32_t ulOffset = ( uint32_t ) eFirst - ( uint32_t ) eStatPoolCritEmpty;
    uint32_t ulFree = STATS_MAX_POOLS;
    uint32_t x;

    /* eFirst must start one of the three groups: offset 0, 3 or 6 from
     * eStatPoolCritEmpty. Below eStatPoolCritEmpty the unsigned offset wraps
     * to a huge value and fails the range check. */
    if( ( pxPool != NULL ) &&
        ( ulOffset < ( ( uint32_t ) eStatPoolUnregistered - ( uint32_t ) eStatPoolCritEmpty ) ) &&
        ( ( ulOffset % ( uint32_t ) eStatPoolEvtCount ) == 0U ) )
    {
        bOk = true;

        for( x = 0U; x < STATS_MAX_POOLS; x++ )
        {
            if( xPools[ x ].pxPool == NULL )
            {
                if( ulFree == STATS_MAX_POOLS )
                {
                    ulFree = x;                         /* first free slot  */
                }
            }
            else if( ( xPools[ x ].pxPool == pxPool ) || ( xPools[ x ].eFirst == eFirst ) )
            {
                bOk = false;                            /* registered twice */
            }
            else
            {
                /* another pool with another group */
            }
        }

        if( bOk && ( ulFree < STATS_MAX_POOLS ) )
        {
            xPools[ ulFree ].eFirst = eFirst;
            xPools[ ulFree ].pxPool = pxPool;
        }
        else
        {
            bOk = false;
        }
    }

    configASSERT( bOk );

    return bOk;
}

/*---------------------------------------------------------------------------*/
/* Stats_PoolEvent                                                           */
/*---------------------------------------------------------------------------*/
void Stats_PoolEvent( const struct xFIXED_POOL * pxPool, eStatPoolEvent_t eEvent )
{
    eStatId_t eId = eStatPoolUnregistered;
    uint32_t x;

    configASSERT( ( uint32_t ) eEvent < ( uint32_t ) eStatPoolEvtCount );   /* B9 */

    /* At most 3 entries. The table does not change once the scheduler runs,
     * so it is read here without a lock, even from an ISR. */
    if( pxPool != NULL )
    {
        for( x = 0U; x < STATS_MAX_POOLS; x++ )
        {
            if( xPools[ x ].pxPool == pxPool )
            {
                eId = ( eStatId_t ) ( ( uint32_t ) xPools[ x ].eFirst + ( uint32_t ) eEvent );
                break;
            }
        }
    }

    Stats_Inc( eId );
}

/*---------------------------------------------------------------------------*/
/* Stats_Snapshot                                                            */
/*---------------------------------------------------------------------------*/
void Stats_Snapshot( StatsSnapshot_t * pxOut )
{
    uint32_t x;

    if( pxOut != NULL )
    {
        /* 1. Counters and gauges at one instant, without a lock (report
         *    092): copy, then compare with the live values. Counters only
         *    grow and each gauge moves one way only, so a value equal at the
         *    copy and at the check did not change in between, and all of
         *    them held their copied value at the end of the copy: the docs
         *    18.2 equations compare values of one moment. A change during
         *    the copy (an ISR or a task counted) means another round; after
         *    STATS_SNAPSHOT_TRIES the last copy is kept (on a busy link the
         *    equations allow for in-flight items anyway, 18.1 rule 3). */
        for( x = 0U; x < STATS_SNAPSHOT_TRIES; x++ )
        {
            pxOut->ulTick = ( uint32_t ) xTaskGetTickCount();
            ( void ) memcpy( pxOut->ulCounters, ulCounters, sizeof( pxOut->ulCounters ) );
            ( void ) memcpy( pxOut->ulGauges, ulGauges, sizeof( pxOut->ulGauges ) );
            __atomic_thread_fence( __ATOMIC_SEQ_CST );

            if( ( memcmp( pxOut->ulCounters, ulCounters, sizeof( pxOut->ulCounters ) ) == 0 ) &&
                ( memcmp( pxOut->ulGauges, ulGauges, sizeof( pxOut->ulGauges ) ) == 0 ) )
            {
                break;
            }
        }

        /* 2. Latencies: each copied whole by Stats_LatencyGet. */
        for( x = 0U; x < ( uint32_t ) eLatCount; x++ )
        {
            Stats_LatencyGet( ( eLatId_t ) x, &( pxOut->xLat[ x ] ) );
        }

        /* 3. Resource levels, interrupts enabled. */
        prvSampleResources( pxOut );
    }
}

/*---------------------------------------------------------------------------*/
/* prvSampleResources: heap, network buffers, IP queue, pools, stacks        */
/*---------------------------------------------------------------------------*/
static void prvSampleResources( StatsSnapshot_t * pxOut )
{
    uint32_t x;
    uint32_t ulGroup;

    pxOut->ulHeapFree = ( uint32_t ) xPortGetFreeHeapSize();
    pxOut->ulHeapMinFree = ( uint32_t ) xPortGetMinimumEverFreeHeapSize();
    pxOut->ulNbFree = ( uint32_t ) uxGetNumberOfFreeNetworkBuffers();
    pxOut->ulNbMinFree = ( uint32_t ) uxGetMinimumFreeNetworkBuffers();
    pxOut->ulIpqMinFree = ( uint32_t ) uxGetMinimumIPQueueSpace();

    /* Pools, placed by their counter group: crit 0, low 1, report 2. */
    for( x = 0U; x < STATS_MAX_POOLS; x++ )
    {
        pxOut->ulPoolFree[ x ] = STATS_NA;
        pxOut->ulPoolMinFree[ x ] = STATS_NA;
    }

    for( x = 0U; x < STATS_MAX_POOLS; x++ )
    {
        if( xPools[ x ].pxPool != NULL )
        {
            ulGroup = ( ( uint32_t ) xPools[ x ].eFirst - ( uint32_t ) eStatPoolCritEmpty ) /
                      ( uint32_t ) eStatPoolEvtCount;
            pxOut->ulPoolFree[ ulGroup ] = ( uint32_t ) Pool_GetFree( xPools[ x ].pxPool );
            pxOut->ulPoolMinFree[ ulGroup ] = ( uint32_t ) Pool_GetMinFree( xPools[ x ].pxPool );
        }
    }

    /* Stacks: each call scans the task's stack, so this is the slow part. */
    for( x = 0U; x < ( uint32_t ) eAppTaskCount; x++ )
    {
        pxOut->ulStackHwm[ x ] = prvStackHwm( xTaskHandles[ x ] );
    }

    pxOut->ulIpTaskStackHwm = prvStackHwm( FreeRTOS_GetIPTaskHandle() );
    pxOut->ulIdleStackHwm = prvStackHwm( xTaskGetIdleTaskHandle() );
}

/*---------------------------------------------------------------------------*/
/* prvStackHwm: words of xTask's stack never used, STATS_NA without a handle  */
/*---------------------------------------------------------------------------*/
static uint32_t prvStackHwm( TaskHandle_t xTask )
{
    uint32_t ulHwm = STATS_NA;

    /* uxTaskGetStackHighWaterMark( NULL ) would report the CALLING task:
     * a task that is not registered (yet) must not borrow its numbers. */
    if( xTask != NULL )
    {
        ulHwm = ( uint32_t ) uxTaskGetStackHighWaterMark( xTask );
    }

    return ulHwm;
}
