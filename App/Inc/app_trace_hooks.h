/**
  ******************************************************************************
  * @file    app_trace_hooks.h
  * @brief   Every trace hook of the system mapped to app_stats counters (D22):
  *          the iptrace* hooks of FreeRTOS-Plus-TCP and the traceFIXED_POOL_*
  *          hooks of fixed_pool.c.
  ******************************************************************************
  * Read in two places, like FreeRTOSIPConfig.h is read by the stack:
  *   - at the end of Core/Inc/FreeRTOSIPConfig.h: every TCP/IP source sees
  *     the iptrace* definitions before IPTraceMacroDefaults.h, whose empty
  *     defaults are guarded by #ifndef, so these win;
  *   - at the top of App/Inc/fixed_pool.h: fixed_pool.c sees the pool hooks
  *     before its own empty defaults.
  * Each hook is one Stats_Inc / Stats_PoolEvent call: lock-free, never waits,
  * safe in any context the hook fires in (tasks, IP task, ISRs).
  */
#ifndef APP_TRACE_HOOKS_H
#define APP_TRACE_HOOKS_H

#include "app_stats.h"

/*----------------------------------------------------------------------------
 * FreeRTOS-Plus-TCP (IPTraceMacroDefaults.h). Line numbers: V4.4.1 sources.
 *--------------------------------------------------------------------------*/

/* BufferAllocation_1.c:318 - no network buffer within the block time */
#define iptraceFAILED_TO_OBTAIN_NETWORK_BUFFER()            Stats_Inc( eStatNbGetFail )

/* FreeRTOS_Sockets.c:1550 - FreeRTOS_sendto() found no network buffer */
#define iptraceNO_BUFFER_FOR_SENDTO()                       Stats_Inc( eStatSendtoNoBuf )

/* FreeRTOS_IP.c:1388, 1485, FreeRTOS_Sockets.c:1463 - event not queued to
 * the IP task (queue full) */
#define iptraceSTACK_TX_EVENT_LOST( xEvent )                Stats_Inc( eStatIpqEventLost )

/* FreeRTOS_UDP_IPv4.c:272 - outgoing UDP payload replaced by an ARP request */
#define iptracePACKET_DROPPED_TO_GENERATE_ARP( ulIPAddress )    Stats_Inc( eStatArpTxDropped )

/* FreeRTOS_IP.c:1808 - incoming packet held until its sender is resolved */
#define iptraceDELAYED_ARP_REQUEST_STARTED()                Stats_Inc( eStatArpDelayed )

/* FreeRTOS_IP.c:1816 - a second packet arrived while one is held: dropped */
#define iptraceDELAYED_ARP_BUFFER_FULL()                    Stats_Inc( eStatArpDelayedFull )

/* FreeRTOS_ARP.c:1134 - an ARP cache entry aged out */
#define iptraceARP_TABLE_ENTRY_EXPIRED( ulIPAddress )       Stats_Inc( eStatArpExpired )

/* FreeRTOS_IP.c:295 - the IP task took an event from its queue. eEvent is an
 * eIPEvent_t, defined in FreeRTOS_IP_Private.h where this hook fires. */
#define iptraceNETWORK_EVENT_RECEIVED( eEvent )                          \
    Stats_Inc( ( ( eEvent ) == eNetworkRxEvent ) ? eStatIpEvtRx :        \
               ( ( ( eEvent ) == eStackTxEvent ) ? eStatIpEvtTx :        \
                 ( ( ( eEvent ) == eARPTimerEvent ) ? eStatIpEvtArpTimer : \
                   eStatIpEvtOther ) ) )

/*----------------------------------------------------------------------------
 * fixed_pool.c (fixed_pool.h). The pool tells which pool; Stats_PoolEvent
 * finds its counter group in the table Stats_RegisterPool filled (D23).
 *--------------------------------------------------------------------------*/
#define traceFIXED_POOL_GET_FAILED( pxPool )                Stats_PoolEvent( ( pxPool ), eStatPoolEvtGetFailed )
#define traceFIXED_POOL_INVALID_BLOCK( pxPool, pvBlock )    Stats_PoolEvent( ( pxPool ), eStatPoolEvtInvalid )
#define traceFIXED_POOL_DOUBLE_RELEASE( pxPool, pvBlock )   Stats_PoolEvent( ( pxPool ), eStatPoolEvtDouble )

#endif /* APP_TRACE_HOOKS_H */
