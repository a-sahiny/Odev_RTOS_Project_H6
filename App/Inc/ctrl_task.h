/**
  ******************************************************************************
  * @file    ctrl_task.h
  * @brief   ctrlTask (P6): network start-up, sockets, boot report, the
  *          dispatcher loop with Cmd_Handle and the 100 ms health check
  *          (docs 4, 6, 19). Step 15.
  ******************************************************************************
  */
#ifndef CTRL_TASK_H
#define CTRL_TASK_H

#include <stdint.h>

#include "FreeRTOS.h"

/* App_Init, before the scheduler, after AppEvt_Init. Creates ctrlTask
 * (static). Always pdPASS. */
BaseType_t CtrlTask_Init( void );

/* APP_FAULT_* conditions that hold right now, as the last health check saw
 * them (0 = none). ctrlTask only (cmd.c, START). */
uint32_t CtrlTask_GetActiveFaults( void );

#endif /* CTRL_TASK_H */
