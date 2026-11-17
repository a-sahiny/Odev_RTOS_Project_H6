/**
  ******************************************************************************
  * @file    sys_state.c
  * @brief   System state: event group, fault reasons, EPOCH (docs 12, 16.6, 19).
  ******************************************************************************
  * RTOS object (one):
  *   xSysEvents   static EventGroup, 3 of 24 bits used.
  *                NET_READY  writer: IP task (network event hook);
  *                           waiters: ctrlTask (1000 ms loop), meas (ALL, 100 ms)
  *                MEAS_EN    writer: ctrlTask;  waiter: meas (ALL, 100 ms)
  *                FAULT      writer: ctrlTask;  waiters: none (read only)
  *                No clear-on-exit anywhere: the bits are states, not events.
  *                No ...FromISR: every writer is a task.
  * Shared data:
  *   ulFaultReasons  writer: ctrlTask only; readers: everyone (aligned 32-bit
  *                   word, a read is never torn).
  *   usEpoch         written once in SysState_Init, read-only afterwards.
  */
#include "sys_state.h"

#include "FreeRTOS.h"
#include "task.h"           /* configASSERT uses taskDISABLE_INTERRUPTS */
#include "event_groups.h"

#include "FreeRTOS_IP.h"    /* xApplicationGetRandomNumber (net_app.c) */

#include "app_config.h"
#include "app_stats.h"      /* eStatFaultRaised */

/* The STATE byte of every reply (docs 15.3) is the event bits themselves:
 * STATE = bits & 0x07, no translation table. */
_Static_assert( APP_BIT_NET_READY == APP_STATE_BIT_NET_READY, "STATE bit0 == NET_READY event bit" );
_Static_assert( APP_BIT_MEAS_EN == APP_STATE_BIT_MEAS_EN, "STATE bit1 == MEAS_EN event bit" );
_Static_assert( APP_BIT_FAULT == APP_STATE_BIT_FAULT, "STATE bit2 == FAULT event bit" );
_Static_assert( ( APP_BIT_NET_READY | APP_BIT_MEAS_EN | APP_BIT_FAULT ) == 0x07U,
                "the three state bits are bits 0..2" );

/* The top byte of EventBits_t belongs to the kernel (event_groups.h:48-51);
 * xEventGroupSetBits / WaitBits assert on it. */
_Static_assert( ( ( APP_BIT_NET_READY | APP_BIT_MEAS_EN | APP_BIT_FAULT ) & eventEVENT_BITS_CONTROL_BYTES ) == 0U,
                "state bits outside the kernel control byte" );

static StaticEventGroup_t xSysEventsBuf;
static EventGroupHandle_t xSysEvents = NULL;   /* NULL = SysState_Init not called yet */
static uint32_t ulFaultReasons;                 /* APP_FAULT_* bits                     */
static uint16_t usEpoch;

/*---------------------------------------------------------------------------*/
void SysState_Init( void )
{
    uint32_t ulRandom = 0U;
    BaseType_t xGotRandom;

    if( xSysEvents == NULL )
    {
        /* Static storage: cannot fail; the kernel itself asserts a NULL
         * buffer (event_groups.c; report 062, B9). The group starts with
         * every bit 0: no network yet, no measurement (MEAS_EN is clear after
         * every reset, docs 16.6), no fault. */
        xSysEvents = xEventGroupCreateStatic( &xSysEventsBuf );

        ulFaultReasons = 0U;

        /* 16 bits of the 32-bit random number, both halves folded in. */
        xGotRandom = xApplicationGetRandomNumber( &ulRandom );
        configASSERT( xGotRandom == pdTRUE );
        ( void ) xGotRandom;
        usEpoch = ( uint16_t ) ( ulRandom ^ ( ulRandom >> 16 ) );
    }
}

/*---------------------------------------------------------------------------*/
void SysState_SetNetReady( BaseType_t xUp )
{
    /* The kernel asserts on a NULL handle (event_groups.c:482, :562), i.e.
     * on a call before SysState_Init. */
    if( xUp != pdFALSE )
    {
        ( void ) xEventGroupSetBits( xSysEvents, APP_BIT_NET_READY );
    }
    else
    {
        ( void ) xEventGroupClearBits( xSysEvents, APP_BIT_NET_READY );
    }
}

/*---------------------------------------------------------------------------*/
BaseType_t SysState_SetMeasEnabled( BaseType_t xOn )
{
    BaseType_t xResult = pdPASS;

    if( xOn != pdFALSE )
    {
        /* Read FAULT, then set MEAS_EN: two kernel calls, not one atomic
         * step. Safe because ctrlTask, the caller, is also the only writer
         * of FAULT: nobody can set FAULT between the two calls. */
        if( ( xEventGroupGetBits( xSysEvents ) & APP_BIT_FAULT ) != 0U )
        {
            xResult = pdFAIL;
        }
        else
        {
            ( void ) xEventGroupSetBits( xSysEvents, APP_BIT_MEAS_EN );
        }
    }
    else
    {
        ( void ) xEventGroupClearBits( xSysEvents, APP_BIT_MEAS_EN );
    }

    return xResult;
}

/*---------------------------------------------------------------------------*/
void SysState_RaiseFault( uint32_t ulReasons )
{
    EventBits_t uxBefore;

    /* A FAULT without a reason means nothing (docs 19). */
    configASSERT( ulReasons != 0U );

    if( ulReasons != 0U )
    {
        /* 1. Reasons first: whoever sees FAULT must also see why. Only
         *    ctrlTask writes ulFaultReasons, so the read-modify-write needs
         *    no lock. */
        ulFaultReasons |= ulReasons;

        /* 2. Stop measurement. ClearBits returns the bits as they were
         *    BEFORE clearing (event_groups.c:485-496), which also tells
         *    whether FAULT was already set. */
        uxBefore = xEventGroupClearBits( xSysEvents, APP_BIT_MEAS_EN );

        /* 3. Declare the fault, only on the 0 -> 1 edge: a second raise
         *    only added reasons above. */
        if( ( uxBefore & APP_BIT_FAULT ) == 0U )
        {
            ( void ) xEventGroupSetBits( xSysEvents, APP_BIT_FAULT );
            Stats_Inc( eStatFaultRaised );
        }
    }
}

/*---------------------------------------------------------------------------*/
BaseType_t SysState_TryClearFault( uint32_t ulActiveReasons )
{
    BaseType_t xResult = pdFAIL;

    if( ulActiveReasons == 0U )
    {
        /* Reverse order of the raise: FAULT first, then the reasons. Nobody
         * can then see FAULT set with no reason. */
        ( void ) xEventGroupClearBits( xSysEvents, APP_BIT_FAULT );
        ulFaultReasons = 0U;
        xResult = pdPASS;
    }

    return xResult;
}

/*---------------------------------------------------------------------------*/
EventBits_t SysState_Get( void )
{
    EventBits_t uxBits = 0U;

    if( xSysEvents != NULL )
    {
        /* = xEventGroupClearBits( xSysEvents, 0 ) (event_groups.h:765). */
        uxBits = xEventGroupGetBits( xSysEvents );
    }

    return uxBits;
}

/*---------------------------------------------------------------------------*/
uint32_t SysState_GetFaultReasons( void )
{
    /* One aligned 32-bit load; its only writer is ctrlTask. */
    return ulFaultReasons;
}

/*---------------------------------------------------------------------------*/
uint16_t SysState_GetEpoch( void )
{
    return usEpoch;
}

/*---------------------------------------------------------------------------*/
BaseType_t SysState_WaitNetReady( TickType_t xTimeout )
{
    EventBits_t uxBits;

    configASSERT( xTimeout <= APP_MS( APP_NET_READY_WAIT_MS ) );

    /* pdFALSE: no clear on exit. pdTRUE: wait for all requested bits (one
     * bit here, so ANY would be the same). */
    uxBits = xEventGroupWaitBits( xSysEvents, APP_BIT_NET_READY, pdFALSE, pdTRUE, xTimeout );

    /* The returned bits decide, not "did it time out": on a timeout the
     * kernel tests the condition once more (event_groups.c:421-450). */
    return ( ( uxBits & APP_BIT_NET_READY ) != 0U ) ? pdPASS : pdFAIL;
}

/*---------------------------------------------------------------------------*/
BaseType_t SysState_WaitMeasAllowed( TickType_t xTimeout )
{
    const EventBits_t uxWanted = APP_BIT_NET_READY | APP_BIT_MEAS_EN;
    EventBits_t uxBits;

    configASSERT( xTimeout <= APP_MS( APP_MEAS_WAIT_MS ) );

    /* ALL: network AND permission (docs 12, assignment item 14). */
    uxBits = xEventGroupWaitBits( xSysEvents, uxWanted, pdFALSE, pdTRUE, xTimeout );

    return ( ( uxBits & uxWanted ) == uxWanted ) ? pdPASS : pdFAIL;
}
