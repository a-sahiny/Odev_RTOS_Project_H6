/**
  ******************************************************************************
  * @file    app_evt.h
  * @brief   Generic event mechanism (docs 6): the two event queues
  *          (ctrlQueue, workQueue), the two descriptor pools and the
  *          dispatcher loop shared by ctrlTask and worker. Modelled on the IP
  *          task: xNetworkEventQueue, the network buffer pool,
  *          xSendEventStructToIPTask, prvProcessIPEventsAndTimers.
  ******************************************************************************
  * Künye: report 056.
  */
#ifndef APP_EVT_H
#define APP_EVT_H

#include "FreeRTOS.h"

#include "app_types.h"      /* AppDesc_t, eAppClass_t */

/* App_Init, before the scheduler. Creates ctrlQueue and workQueue (static)
 * and sets up the two descriptor pools (fixed_pool, docs 5.2) with their
 * counter groups. A second call after a success does nothing (E10).
 * pdFAIL only for a configuration error in this file (pool settings or a
 * pool counter group already taken); App_Init then stops. */
BaseType_t AppEvt_Init( void );

/* Parsers. A free descriptor of the lane's pool without waiting, or NULL
 * if the pool is empty (counted as CRIT_DESC_EMPTY / LOW_DESC_EMPTY by the
 * pool hook) or before AppEvt_Init. ucClass is set; every other field is
 * the caller's to fill. The payload capacity is APP_CRIT_DESC_DATA_BYTES or
 * APP_LOW_DESC_DATA_BYTES. */
AppDesc_t * AppDesc_Get( eAppClass_t eClass );

/* The dispatcher after a handler, or a parser whose AppEvt_Send failed
 * (docs 6.3). Returns the descriptor to the pool named by its ucClass (E6).
 * NULL does nothing. A foreign block or a second release is caught and
 * counted by fixed_pool. Never waits. */
void AppDesc_Release( AppDesc_t * pxDesc );

/* Parsers and the network hook (IP task); never from an ISR. Copies
 * *pxEvent to the back of the target's queue (ctrlQueue / workQueue),
 * waiting at most xTimeout; every caller passes 0 (docs 4). If the calling
 * task is the receiver itself, the wait is forced to 0: a task cannot wait
 * for itself to make room (E7, as FreeRTOS_IP.c:1468-1476).
 * pdPASS: the event is queued and the descriptor in pvData now belongs to
 * the receiver. pdFAIL: queue full (or bad arguments / before AppEvt_Init,
 * which also assert); the descriptor stays with the caller, who releases it
 * and counts the loss (E3, docs 6.3). */
BaseType_t AppEvt_Send( eAppTarget_t eTarget, const AppEvent_t * pxEvent, TickType_t xTimeout );

/* The queue of a target, for the AppEvtLoop_t of the task that reads it
 * (ctrlTask, worker). NULL before AppEvt_Init (report 069 K1). */
QueueHandle_t AppEvt_GetQueue( eAppTarget_t eTarget );

/* The loop's task, before AppEvt_RunLoop (as prvIPTimerReload,
 * FreeRTOS_IP_Timers.c:449-453). Sets the period and starts the timer now:
 * pfRun is first due xPeriod ticks later, then every xPeriod ticks counted
 * from the check that ran it. xPeriod > 0 (0 would make the loop spin). */
void AppTimer_Start( AppTimer_t * pxTimer, TickType_t xPeriod );

/* ctrlTask / worker: never returns. Each turn, as the IP task's
 * prvProcessIPEventsAndTimers (FreeRTOS_IP.c:255-486): run the due timers,
 * wait on the loop's queue for at most min( xMaxSleep, next timer ), then
 * give the event to its handler and release the descriptor (docs 6.3). */
void AppEvt_RunLoop( const AppEvtLoop_t * pxLoop );

#endif /* APP_EVT_H */
