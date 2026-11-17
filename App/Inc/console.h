/**
  ******************************************************************************
  * @file    console.h
  * @brief   Console commands HELLO / STATE / HOLD / TXDELAY (docs 13.5) and the
  *          state of the artificial injections they switch on (docs 20).
  ******************************************************************************
  * No RTOS object. line_rx decodes the line, report.c writes the answer;
  * console.c decides. Counting is done by the caller (vnetRx, decision L1).
  */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>

#include "line_rx.h"
#include "report.h"         /* ReportInject_t */

/* vnetRx only. Carries out one console line: eLineHello, eLineState,
 * eLineHold, eLineTxDelay, eLineUnknown (-> error unknown_command) and
 * eLineBadArg (-> error bad_arg). FRAME kinds are vnet_if's, not handled here
 * (configASSERT). NULL is ignored. Never waits. */
void Console_Handle( const HostLine_t * pxLine );

/* lowParser only, before every record (docs 13.5, T03). Returns the pending
 * HOLD in ms and clears it in the same atomic step (0: no hold); the caller
 * then waits that long (vTaskDelay). Rules (C4): the last HOLD wins,
 * HOLD 0 cancels a pending one, a running hold is not cut short. */
uint32_t Console_TakeHoldMs( void );

/* The active injections for the "inj" field of st / state (decision C2):
 * mask bit 0 = HOLD pending or running, bit 1 = TXDELAY on; their ms. Any
 * task. NULL is ignored. */
void Console_GetInject( ReportInject_t * pxOut );

#endif /* CONSOLE_H */
