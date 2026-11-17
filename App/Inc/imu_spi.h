/**
  ******************************************************************************
  * @file    imu_spi.h
  * @brief   The IMU (WHO_AM_I 0x74, MPU-6515 compatible) on SPI2: start-up,
  *          DRDY wait, one DMA sample (docs 3.3). Step 17, künye report 076.
  ******************************************************************************
  * One caller: the meas task. The ISR -> task path is meas's own task
  * notification (index 0, eSetBits).
  */
#ifndef IMU_SPI_H
#define IMU_SPI_H

#include "FreeRTOS.h"
#include "task.h"

#include "app_types.h"      /* Sample_t */

/* Resets and configures the IMU (polled SPI, <= 1 MHz), then switches SPI2
 * to the read speed and enables DRDY. xNotifyTask: the task the ISRs wake
 * (meas, the caller). pdFAIL if WHO_AM_I or the read-back is wrong
 * (IMU_WHO_AM_I counted). Task context only; takes ~120 ms. */
BaseType_t Imu_Init( TaskHandle_t xNotifyTask );

/* Drops a DRDY notification left from before the configuration (a board
 * reset does not reset the IMU). Call right after a successful Imu_Init. */
void Imu_DiscardStaleDrdy( void );

/* Waits for the next DRDY, at most xTimeout (<= APP_IMU_DRDY_TIMEOUT_MS).
 * pdFAIL on timeout (IMU_DRDY_TIMEOUT counted). */
BaseType_t Imu_WaitDrdy( TickType_t xTimeout );

/* Reads accel, temperature, gyro (0x3B..0x48) with SPI2 DMA, at most
 * xTimeout (<= APP_IMU_SPI_TIMEOUT_MS). ulTick = the last DRDY's tick;
 * ulIndex is the caller's. pdFAIL on timeout or DMA error
 * (IMU_SPI_TIMEOUT counted); *pxOut untouched then. */
BaseType_t Imu_ReadSample( Sample_t * pxOut, TickType_t xTimeout );

#endif /* IMU_SPI_H */
