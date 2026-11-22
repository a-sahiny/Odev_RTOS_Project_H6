/**
  ******************************************************************************
  * @file    ctrl_task.c
  * @brief   ctrlTask (P6): start-up (NET_READY, sockets, boot report), the
  *          generic dispatcher loop with Cmd_Handle, the 100 ms health check
  *          (docs 4, 6, 12, 19). Künye: report 069.
  ******************************************************************************
  * RTOS object owned:
  *   ctrlTask  static task, APP_PRIO_CTRL (6), APP_STACK_CTRL_WORDS. Waits on
  *             NET_READY (<= APP_NET_READY_WAIT_MS per round) and FreeRTOS_bind
  *             at start-up only, then on ctrlQueue (<= APP_CTRL_MAX_SLEEP_MS).
  * Used, not owned: ctrlQueue (the only reader) and the crit descriptor pool
  * (app_evt.c), xSysEvents (sys_state.c: the only writer of MEAS_EN and
  * FAULT), the UDP sockets (udp_in.c).
  * Not RTOS objects: the loop description, the health timer and the health
  * state; ctrlTask alone touches them.
  */
#include "ctrl_task.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "FreeRTOS_IP.h"
#include "NetworkBufferManagement.h"    /* uxGetNumberOfFreeNetworkBuffers (POOL) */

#include "app_config.h"
#include "app_evt.h"
#include "app_stats.h"
#include "cmd.h"
#include "report.h"
#include "sys_state.h"
#include "udp_in.h"

static StaticTask_t xCtrlTcb;
static StackType_t xCtrlStack[ APP_STACK_CTRL_WORDS ];
static TaskHandle_t xCtrlTask = NULL;          /* NULL = not created yet */

static eAppDescAction_t prvOnNetUp( void * pvData );
static eAppDescAction_t prvOnNetDown( void * pvData );
static void prvCtrlTask( void * pvParameters );
static void prvCtrlInitialise( void );
static void prvHealthCheck( void );
static uint32_t prvEvalFaults( void );

/* docs 6.2: the events ctrlTask handles; the others are counted as
 * EVT_NO_HANDLER by the loop. */
static const AppEvtHandler_t xCtrlHandlers[ eAppEventCount ] =
{
    [ eAppCmdEvent ]     = Cmd_Handle,
    [ eAppNetUpEvent ]   = prvOnNetUp,
    [ eAppNetDownEvent ] = prvOnNetDown
};

/* The health check (docs 19), run by the loop every APP_HEALTH_PERIOD_MS. */
static AppTimer_t xCtrlTimers[ 1 ] = { { 0U, 0U, { 0, 0U }, 0U, 0U, prvHealthCheck } };

/* The queue handle exists only after AppEvt_Init: written once at the start
 * of the task, constant afterwards (K2). */
static AppEvtLoop_t xCtrlLoop =
{
    NULL, xCtrlHandlers, xCtrlTimers, 1U, APP_MS( APP_CTRL_MAX_SLEEP_MS ), eStatEvtHandledCtrl, eGaugeCtrlqMinFree
};

static uint32_t ulActiveFaults;                 /* the last health check's result */

/* Checks in one second: the UART_ORE window (docs 19: "1 s içinde"). */
#define CTRL_ORE_CHECKS     ( 1000U / APP_HEALTH_PERIOD_MS )

/* What the health check remembers between two runs (ctrlTask only). */
typedef struct xCTRL_HEALTH
{
    uint32_t ulWho;                             /* counters at the last check */
    uint32_t ulFailRuns;
    uint32_t ulOre;
    uint32_t ulStall;
    uint32_t ulOreRing[ CTRL_ORE_CHECKS ];      /* overruns per period, last second */
    uint8_t ucOreIdx;
    uint8_t ucPoolLowRuns;                      /* checks in a row with few buffers */
} CtrlHealth_t;

static CtrlHealth_t xHealth;

_Static_assert( sizeof( "ctrlTask" ) <= configMAX_TASK_NAME_LEN, "task name fits" );
_Static_assert( APP_HEALTH_PERIOD_MS <= APP_CTRL_MAX_SLEEP_MS, "the loop wakes up for the health check" );
_Static_assert( ( CTRL_ORE_CHECKS > 0U ) && ( CTRL_ORE_CHECKS <= UINT8_MAX ), "ORE window index is 8 bits" );

/*---------------------------------------------------------------------------*/
BaseType_t CtrlTask_Init( void )
{
    if( xCtrlTask == NULL )
    {
        /* Static TCB and stack: the kernel asserts NULL buffers itself
         * (tasks.c:1276-1290; report 062, B9). */
        xCtrlTask = xTaskCreateStatic( prvCtrlTask, "ctrlTask", APP_STACK_CTRL_WORDS, NULL,
                                       APP_PRIO_CTRL, xCtrlStack, &xCtrlTcb );

        /* For the stack high-water mark in the snapshot; also the receiver
         * handle AppEvt_Send compares with (report 056 E7). */
        Stats_SetTaskHandle( eAppTaskCtrl, xCtrlTask );
    }

    return pdPASS;
}

/*---------------------------------------------------------------------------*/
uint32_t CtrlTask_GetActiveFaults( void )
{
    return ulActiveFaults;
}

/*---------------------------------------------------------------------------*/
static void prvCtrlTask( void * pvParameters )
{
    ( void ) pvParameters;

    prvCtrlInitialise();

    /* docs 6.4: the IP task's loop, ours. Never returns. */
    AppEvt_RunLoop( &xCtrlLoop );
}

/*---------------------------------------------------------------------------*/
/* prvCtrlInitialise: docs 21.4 "SysState_WaitNetReady, UdpIn_OpenSockets,   */
/* Report_Boot". FreeRTOS_bind (inside OpenSockets) waits for the IP task    */
/* without a limit; it is safe only after NET_READY (docs 12 note).          */
/*---------------------------------------------------------------------------*/
static void prvCtrlInitialise( void )
{
    BaseType_t xOpen = pdFAIL;

    Cmd_Init();
    xCtrlLoop.xQueue = AppEvt_GetQueue( eAppTargetCtrl );
    configASSERT( xCtrlLoop.xQueue != NULL );       /* AppEvt_Init in App_Init */

    while( xOpen != pdPASS )
    {
        /* 1000 ms rounds: the task never blocks without a limit here. */
        while( SysState_WaitNetReady( APP_MS( APP_NET_READY_WAIT_MS ) ) != pdPASS )
        {
        }

        xOpen = UdpIn_OpenSockets();

        if( xOpen != pdPASS )
        {
            /* The sockets come from the heap (docs 18.3); try again (K3). */
            Report_Error( "socket_open" );
            vTaskDelay( APP_MS( APP_NET_READY_WAIT_MS ) );
        }
    }

    Report_Boot();

    AppTimer_Start( &xCtrlTimers[ 0 ], APP_MS( APP_HEALTH_PERIOD_MS ) );
}

/*---------------------------------------------------------------------------*/
/* prvOnNetUp / prvOnNetDown: the network hook's events (report 056). The    */
/* state is already in NET_READY and meas waits on it (docs 12), so there is */
/* nothing to do; the events are counted by the loop (L5; K4).               */
/*---------------------------------------------------------------------------*/
static eAppDescAction_t prvOnNetUp( void * pvData )
{
    configASSERT( pvData == NULL );     /* no descriptor in a network event */
    return eAppDescRelease;
}

static eAppDescAction_t prvOnNetDown( void * pvData )
{
    configASSERT( pvData == NULL );
    return eAppDescRelease;
}

/*---------------------------------------------------------------------------*/
/* prvHealthCheck: docs 19. Every APP_HEALTH_PERIOD_MS, in ctrlTask.         */
/*---------------------------------------------------------------------------*/
static void prvHealthCheck( void )
{
    BaseType_t xWasSet;

    ulActiveFaults = prvEvalFaults();

    if( ulActiveFaults != 0U )
    {
        /* FAULT is latched; a reason that comes while it is already set is
         * added but not reported again (K7, docs 19). RaiseFault clears
         * MEAS_EN itself (report 024). */
        xWasSet = ( ( SysState_Get() & APP_BIT_FAULT ) != 0U ) ? pdTRUE : pdFALSE;
        SysState_RaiseFault( ulActiveFaults );

        if( xWasSet == pdFALSE )
        {
            Report_Fault( pdTRUE, SysState_GetFaultReasons() );
        }
    }
}

/*---------------------------------------------------------------------------*/
/* prvEvalFaults: the conditions of docs 19 that hold now (K5, K6). Counter  */
/* differences since the last check; the first check compares with 0, so a  */
/* failure before the timer started (WHO_AM_I at start-up) is seen too.      */
/*---------------------------------------------------------------------------*/
static uint32_t prvEvalFaults( void )
{
    uint32_t ulReasons = 0U;
    uint32_t ulWho = Stats_Get( eStatImuWhoAmI );
    uint32_t ulFailRuns = Stats_Get( eStatImuFailRun );
    uint32_t ulOre = Stats_Get( eStatUartOre );
    uint32_t ulStall = Stats_Get( eStatTxUartStall );
    uint32_t ulOreInWindow = 0U;
    uint32_t x;

    /* IMU: a WHO_AM_I failure, or APP_IMU_FAIL_LIMIT failed reads in a row
     * (meas counts IMU_FAIL_RUN). Counted by meas, not as timeouts per
     * period: at 10 Hz one DRDY timeout lasts 110 ms, longer than a check. */
    if( ( ulWho != xHealth.ulWho ) || ( ulFailRuns != xHealth.ulFailRuns ) )
    {
        ulReasons |= APP_FAULT_IMU;
    }

    /* POOL: fewer than APP_FAULT_POOL_MIN_FREE network buffers in
     * APP_FAULT_POOL_CHECKS checks in a row. */
    if( uxGetNumberOfFreeNetworkBuffers() < APP_FAULT_POOL_MIN_FREE )
    {
        if( xHealth.ucPoolLowRuns < UINT8_MAX )
        {
            xHealth.ucPoolLowRuns++;
        }
    }
    else
    {
        xHealth.ucPoolLowRuns = 0U;
    }

    if( xHealth.ucPoolLowRuns >= APP_FAULT_POOL_CHECKS )
    {
        ulReasons |= APP_FAULT_POOL;
    }

    /* UART_ORE: at least APP_FAULT_ORE_PER_S overruns in the last second
     * (CTRL_ORE_CHECKS periods, a ring of per-period counts). */
    xHealth.ulOreRing[ xHealth.ucOreIdx ] = ulOre - xHealth.ulOre;
    xHealth.ucOreIdx = ( uint8_t ) ( ( xHealth.ucOreIdx + 1U ) % CTRL_ORE_CHECKS );

    for( x = 0U; x < CTRL_ORE_CHECKS; x++ )
    {
        ulOreInWindow += xHealth.ulOreRing[ x ];
    }

    if( ulOreInWindow >= APP_FAULT_ORE_PER_S )
    {
        ulReasons |= APP_FAULT_UART_ORE;
    }

    /* UART_TX_STALL: any stall since the last check ("1 kez"). */
    if( ulStall != xHealth.ulStall )
    {
        ulReasons |= APP_FAULT_UART_TX_STALL;
    }

    /* HEAP: free heap below APP_FAULT_HEAP_MIN_BYTES (only the stack uses
     * the heap, docs 18.3). */
    if( xPortGetFreeHeapSize() < APP_FAULT_HEAP_MIN_BYTES )
    {
        ulReasons |= APP_FAULT_HEAP;
    }

    xHealth.ulWho = ulWho;
    xHealth.ulFailRuns = ulFailRuns;
    xHealth.ulOre = ulOre;
    xHealth.ulStall = ulStall;

    return ulReasons;
}
