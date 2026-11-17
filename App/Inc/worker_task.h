/**
  ******************************************************************************
  * @file    worker_task.h
  * @brief   worker (P1): ENV statistics, BULK pattern check, CAL hand-over to
  *          meas, the 1 s st / env reports (docs 5.2, 17). Step 16.
  ******************************************************************************
  */
#ifndef WORKER_TASK_H
#define WORKER_TASK_H

#include <stddef.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "report.h"         /* ReportEnvChannel_t */

/* App_Init, before the scheduler, after AppEvt_Init. Creates worker
 * (static) and the CAL mailbox. Always pdPASS. */
BaseType_t WorkerTask_Init( void );

/* A consistent copy of the ENV channel summaries; returns the number of
 * channels written (<= uxMax, <= APP_ENV_MAX_CHANNELS). Any task. */
size_t Worker_GetEnvStats( ReportEnvChannel_t * pxOut, size_t uxMax );

/* The CAL mailbox (length 1, ImuCal_t, written with xQueueOverwrite).
 * meas looks into it with a 0 wait (docs 3.3, 17.2). Valid after
 * WorkerTask_Init. */
QueueHandle_t Worker_GetCalMailbox( void );

#endif /* WORKER_TASK_H */
