/**
  ******************************************************************************
  * @file    sys_state.h
  * @brief   System state (docs 12, 16.6, 19): the event group with the three
  *          state bits NET_READY / MEAS_EN / FAULT, the fault reasons and the
  *          EPOCH. sys_state.c is the only owner; nobody else touches the
  *          event group directly.
  ******************************************************************************
  * Bit owners (one writer task per bit, docs 12):
  *   NET_READY  IP task (network event hook)
  *   MEAS_EN    ctrlTask (START / STOP, cleared by a FAULT)
  *   FAULT      ctrlTask (health check / START)
  */
#ifndef SYS_STATE_H
#define SYS_STATE_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "event_groups.h"

/* Creates the event group (static, all bits 0), clears the fault reasons and
 * picks the EPOCH. Call once from App_Init, before the scheduler starts. A
 * second call changes nothing (the EPOCH must stay the same for the whole
 * run). */
void SysState_Init( void );

/* NET_READY = xUp (pdFALSE clears it). Only the network event hook calls it,
 * i.e. only the IP task. Never waits; setting the bit wakes ctrlTask / meas
 * when their wait condition becomes true. Task context only. */
void SysState_SetNetReady( BaseType_t xUp );

/* MEAS_EN = xOn. Only ctrlTask calls it (START / STOP). Turning it on is
 * refused while FAULT is set: returns pdFAIL and MEAS_EN stays clear.
 * Turning it off always succeeds. Returns pdPASS otherwise (also when the bit
 * already had that value). Never waits. Task context only. */
BaseType_t SysState_SetMeasEnabled( BaseType_t xOn );

/* Raises FAULT for ulReasons (APP_FAULT_* bits, must not be 0). Only
 * ctrlTask calls it (health check). Order: reasons are added, then MEAS_EN
 * is cleared, then FAULT is set, so whoever sees FAULT also sees its
 * reasons and measurement has already stopped. FAULT is latched: it stays
 * until SysState_TryClearFault. A raise while FAULT is already set only adds
 * the new reasons. Never waits. Task context only. */
void SysState_RaiseFault( uint32_t ulReasons );

/* Called by ctrlTask on START with the fault conditions active right now
 * (APP_FAULT_* bits). ulActiveReasons != 0: returns pdFAIL, FAULT and its
 * reasons stay (START gets E_STATE). ulActiveReasons == 0: clears FAULT,
 * then the reasons, returns pdPASS (also when there was no FAULT). Never
 * waits. Task context only. */
BaseType_t SysState_TryClearFault( uint32_t ulActiveReasons );

/* Read functions (task context, never wait). Before SysState_Init they
 * return 0. */

/* The state bits right now. Bits 0..2 are the STATE byte of every reply
 * (APP_STATE_BIT_*): STATE = ( uint8_t ) SysState_Get(). */
EventBits_t SysState_Get( void );

/* The APP_FAULT_* reasons of the current FAULT; 0 when there is no FAULT. */
uint32_t SysState_GetFaultReasons( void );

/* The EPOCH chosen in SysState_Init. */
uint16_t SysState_GetEpoch( void );

/* Wait functions (task context). Neither clears a bit on exit: the bits are
 * states, other tasks see them too. */

/* ctrlTask at start-up: blocks until NET_READY is set, at most xTimeout
 * ticks (<= APP_NET_READY_WAIT_MS). pdPASS if the network is up on return,
 * pdFAIL on timeout; the caller loops. */
BaseType_t SysState_WaitNetReady( TickType_t xTimeout );

/* meas: blocks until NET_READY AND MEAS_EN are both set, at most xTimeout
 * ticks (<= APP_MEAS_WAIT_MS). pdPASS if measuring is allowed on return,
 * pdFAIL on timeout. */
BaseType_t SysState_WaitMeasAllowed( TickType_t xTimeout );

#endif /* SYS_STATE_H */
