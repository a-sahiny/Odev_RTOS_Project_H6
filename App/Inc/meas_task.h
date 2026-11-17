/**
  ******************************************************************************
  * @file    meas_task.h
  * @brief   The measurement task meas (docs 3.3): IMU sample -> CAL ->
  *          decimation -> TELEM. Step 17, künye report 079.
  ******************************************************************************
  */
#ifndef MEAS_TASK_H
#define MEAS_TASK_H

#include <stdint.h>

#include "FreeRTOS.h"

/* App_Init, before the scheduler, after WorkerTask_Init (meas takes the CAL
 * mailbox at its start). Creates meas (static). Always pdPASS. */
BaseType_t Meas_Init( void );

/* ctrlTask (START): where telemetry goes, both in network byte order as the
 * stack gives them. One writer (ctrlTask), one reader (meas); meas never
 * sees the IP of one START with the port of another (critical section on
 * both sides, report 079 K1). Each call also starts a new measurement:
 * sample index and decimation begin again. */
void Meas_SetDestination( uint32_t ulIpNet, uint16_t usPortNet );

/* ctrlTask (SET_RATE): telemetry rate, APP_RATE_MIN_HZ..APP_RATE_MAX_HZ
 * (cmd.c validates; assert only); takes effect at the next sample. */
void Meas_SetRateHz( uint16_t usHz );

/* The telemetry rate in force (any task). */
uint16_t Meas_GetRateHz( void );

struct xIMU_ATTITUDE;

/* ctrlTask (GET_IMU, report 086): a copy of the latest sample with its
 * roll / pitch / yaw, taken inside a critical section (meas writes it at
 * every sample). pdFAIL while the measurement of the current START has no
 * sample yet; the copy is then not written. */
BaseType_t Meas_GetAttitude( struct xIMU_ATTITUDE * pxOut );

#endif /* MEAS_TASK_H */
