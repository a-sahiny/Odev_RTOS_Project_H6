/**
  ******************************************************************************
  * @file    cmd.h
  * @brief   Command handler of ctrlTask (docs 16): START, STOP, SET_RATE,
  *          GET_STATS; validation, execution, duplicate window, reply.
  ******************************************************************************
  * Künye: report 065.
  */
#ifndef CMD_H
#define CMD_H

#include "app_types.h"      /* eAppDescAction_t */

/* ctrlTask, once, before its loop: empties the duplicate window and the
 * reply SEQ. */
void Cmd_Init( void );

/* ctrlTask's handler for eAppCmdEvent (docs 6.2). pvData is an AppDesc_t *
 * of the crit pool, validated by critParser (TYPE, LEN; report 060 P3).
 * Processes it in the order of docs 16.3 and sends the reply with
 * FreeRTOS_sendto (<= APP_CTRL_SEND_TIMEOUT_MS). Always eAppDescRelease. */
eAppDescAction_t Cmd_Handle( void * pvData );

#endif /* CMD_H */
