/**
  ******************************************************************************
  * @file    meas_task.h
  * @brief   The measurement task meas (docs 3.3): one IMU sample at every
  *          DRDY -> CAL -> attitude -> TELEM. Step 17, künye report 079.
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
 * sees the IP of one START with the port of another (mailbox, report 092).
 * Each call also starts a new measurement: the attitude begins again. */
void Meas_SetDestination( uint32_t ulIpNet, uint16_t usPortNet );

/* pdTRUE if usHz is one of APP_RATE_LIST_HZ (cmd.c, SET_RATE). */
BaseType_t Meas_IsRateValid( uint16_t usHz );

/* ctrlTask (SET_RATE): the IMU sample rate = the TELEM rate, one of
 * APP_RATE_LIST_HZ (cmd.c validates; assert only). meas sets the IMU to it
 * before its next sample. */
void Meas_SetRateHz( uint16_t usHz );

/* The rate SET_RATE asked for (any task). */
uint16_t Meas_GetRateHz( void );

/* uartTx: the DRDY time (DWT cycles) of the TELEM with header SEQ usSeq, kept
 * by meas just before it sent that TELEM. pdFAIL if no TELEM in flight has
 * that SEQ (its slot was taken by a later one). */
BaseType_t Meas_GetTelemDrdy( uint16_t usSeq, uint32_t * pulCycles );

#endif /* MEAS_TASK_H */
