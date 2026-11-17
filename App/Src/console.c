/**
  ******************************************************************************
  * @file    console.c
  * @brief   Console commands and the artificial injections (docs 13.5, 20).
  ******************************************************************************
  * No RTOS object; the only RTOS call is xTaskGetTickCount.
  * Shared data:
  *   xSnap            StatsSnapshot_t (~0.7 KB) for STATE: vnetRx only; static
  *                    because it is too big for vnetRx's stack (C5).
  *   ulHoldPendingMs  TWO writers: vnetRx stores a HOLD, lowParser takes and
  *                    clears it. Atomic store / exchange (LDREX/STREX), so a
  *                    HOLD arriving while lowParser takes the old one is
  *                    never lost.
  *   xHoldStartTick, ulHoldRunMs   written by lowParser only (the hold it
  *                    took); read by the report writers.
  * All are aligned 32-bit words: a plain read is never torn.
  * TXDELAY has one copy only, in uart_tx.c (UartTx_GetInjectedDelay; report
  * 062, B10).
  */
#include "console.h"

#include "FreeRTOS.h"
#include "task.h"           /* xTaskGetTickCount; configASSERT */

#include "app_config.h"
#include "app_stats.h"
#include "report.h"
#include "uart_tx.h"        /* UartTx_Set / GetInjectedDelay (C1, B10) */

/* inj mask bits (docs 13.5) */
#define CONSOLE_INJ_HOLD        0x01U
#define CONSOLE_INJ_TXDELAY     0x02U

static StatsSnapshot_t xSnap;
static uint32_t ulHoldPendingMs;    /* HOLD asked for, not taken yet (atomic) */
static TickType_t xHoldStartTick;   /* when lowParser started the running hold */
static uint32_t ulHoldRunMs;        /* length of the running hold, 0 = none    */

static void prvOnHello( void );
static void prvOnState( void );
static void prvOnHold( uint32_t ulMs );
static void prvOnTxDelay( uint32_t ulMs );

_Static_assert( APP_TXDELAY_MAX_MS <= UINT16_MAX, "UartTx_SetInjectedDelay takes 16 bits" );

/*---------------------------------------------------------------------------*/
void Console_Handle( const HostLine_t * pxLine )
{
    if( pxLine != NULL )
    {
        switch( pxLine->eKind )
        {
            case eLineHello:
                prvOnHello();
                break;

            case eLineState:
                prvOnState();
                break;

            case eLineHold:
                prvOnHold( pxLine->ulValue );
                break;

            case eLineTxDelay:
                prvOnTxDelay( pxLine->ulValue );
                break;

            case eLineUnknown:
                Report_Error( "unknown_command" );     /* C3, docs 13.5 */
                break;

            case eLineBadArg:
                Report_Error( "bad_arg" );             /* C3 */
                break;

            case eLineFrame:
            case eLineBadFlag:
            case eLineBadHex:
            case eLineFrameLength:
            default:
                /* FRAME lines belong to vnet_if (frame checks, rx_err). Reaching
                 * here is the caller's mistake. */
                configASSERT( pxLine->eKind == eLineUnknown );
                break;
        }
    }
}

/*---------------------------------------------------------------------------*/
/* prvOnHello: who and where the board is.                                   */
/*---------------------------------------------------------------------------*/
static void prvOnHello( void )
{
    Report_Hello();
}

/*---------------------------------------------------------------------------*/
/* prvOnState: the resource levels right now (docs 18.3, R2-A). The snapshot */
/* takes one critical section of ~10-15 us (report 019); the report itself  */
/* never waits.                                                              */
/*---------------------------------------------------------------------------*/
static void prvOnState( void )
{
    ReportInject_t xInj;

    Stats_Snapshot( &xSnap );
    Console_GetInject( &xInj );
    Report_State( &xSnap, &xInj );
}

/*---------------------------------------------------------------------------*/
/* prvOnHold: remember the request for lowParser (T03). The command takes    */
/* effect even if the ack report is dropped (no report block).              */
/*---------------------------------------------------------------------------*/
static void prvOnHold( uint32_t ulMs )
{
    /* The limit is enforced once, where the input arrives: line_rx rejects
     * a larger HOLD (BadArg). Here it would be a bug (report 062, B8). */
    configASSERT( ulMs <= APP_HOLD_MAX_MS );

    /* C4: overwrites a pending one; 0 cancels it. */
    __atomic_store_n( &ulHoldPendingMs, ulMs, __ATOMIC_RELAXED );

    Report_Ack( "hold_requested" );
}

/*---------------------------------------------------------------------------*/
/* prvOnTxDelay: uartTx waits ulMs before every item from now on (T06).      */
/*---------------------------------------------------------------------------*/
static void prvOnTxDelay( uint32_t ulMs )
{
    /* line_rx rejects a larger TXDELAY (B8). */
    configASSERT( ulMs <= APP_TXDELAY_MAX_MS );

    UartTx_SetInjectedDelay( ( uint16_t ) ulMs );

    Report_Ack( "txdelay_set" );
}

/*---------------------------------------------------------------------------*/
uint32_t Console_TakeHoldMs( void )
{
    uint32_t ulMs;

    /* Take and clear in one step: a HOLD stored by vnetRx at this moment is
     * either taken now or stays for the next record, never lost. */
    ulMs = __atomic_exchange_n( &ulHoldPendingMs, 0U, __ATOMIC_RELAXED );

    if( ulMs != 0U )
    {
        /* The running hold, for the inj report: length 0, then the start
         * tick, then the length. Plain stores may be reordered by the
         * compiler (the first build did exactly that); the release store
         * keeps the start tick before the new length, and the readers
         * load the length with acquire. */
        __atomic_store_n( &ulHoldRunMs, 0U, __ATOMIC_RELAXED );
        xHoldStartTick = xTaskGetTickCount();
        __atomic_store_n( &ulHoldRunMs, ulMs, __ATOMIC_RELEASE );
    }
    else
    {
        /* lowParser runs again: any earlier hold is over. */
        __atomic_store_n( &ulHoldRunMs, 0U, __ATOMIC_RELAXED );
    }

    return ulMs;
}

/*---------------------------------------------------------------------------*/
void Console_GetInject( ReportInject_t * pxOut )
{
    uint32_t ulPending;
    uint32_t ulRun;
    uint32_t ulHoldMs = 0U;
    uint32_t ulMask = 0U;
    uint32_t ulTxDelayMs;

    if( pxOut != NULL )
    {
        ulTxDelayMs = ( uint32_t ) UartTx_GetInjectedDelay();       /* B10 */
        ulPending = __atomic_load_n( &ulHoldPendingMs, __ATOMIC_RELAXED );
        ulRun = __atomic_load_n( &ulHoldRunMs, __ATOMIC_ACQUIRE );    /* then xHoldStartTick */

        if( ulPending != 0U )
        {
            ulHoldMs = ulPending;
        }
        else if( ( ulRun != 0U ) &&
                 ( ( TickType_t ) ( xTaskGetTickCount() - xHoldStartTick ) < APP_MS( ulRun ) ) )
        {
            /* Still waiting: unsigned difference, right across a tick wrap. */
            ulHoldMs = ulRun;
        }
        else
        {
            /* no hold */
        }

        if( ulHoldMs != 0U )
        {
            ulMask |= CONSOLE_INJ_HOLD;
        }

        if( ulTxDelayMs != 0U )
        {
            ulMask |= CONSOLE_INJ_TXDELAY;
        }

        pxOut->ulMask = ulMask;
        pxOut->ulHoldMs = ulHoldMs;
        pxOut->ulTxDelayMs = ulTxDelayMs;
    }
}
