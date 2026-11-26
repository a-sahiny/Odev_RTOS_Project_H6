/**
  ******************************************************************************
  * @file    imu_spi.h
  * @brief   The IMU (WHO_AM_I 0x74, MPU-6515 compatible) on SPI2: start-up,
  *          DRDY wait, one DMA block of FIFO points (docs 3.3). Step 17, künye report 076.
  ******************************************************************************
  * One caller: the meas task. The ISR -> task path is meas's own task
  * notification (index 0, eSetBits).
  */
#ifndef IMU_SPI_H
#define IMU_SPI_H

#include "FreeRTOS.h"
#include "task.h"

#include "app_types.h"      /* ImuBlock_t */

/* Resets and configures the IMU (SPI2 DMA, <= 1 MHz) for usRateHz (one of
 * APP_RATE_LIST_HZ), then switches SPI2 to the read speed and enables DRDY.
 * xNotifyTask: the task the ISRs wake (meas, the caller). pdFAIL if
 * WHO_AM_I or the read-back is wrong (IMU_WHO_AM_I counted). Task context
 * only; takes ~120 ms. */
BaseType_t Imu_Init( TaskHandle_t xNotifyTask, uint16_t usRateHz );

/* A new sample rate on a running IMU (SET_RATE): SMPLRT_DIV and the gyro /
 * accel DLPF for usRateHz (one of APP_RATE_LIST_HZ), read back. pdFAIL if
 * the read-back is wrong (IMU_WHO_AM_I counted). meas only; ~1 ms. Call
 * Imu_DiscardStaleDrdy after it: the next DRDY is at the new rate. */
BaseType_t Imu_SetRate( uint16_t usRateHz );

/* FIFO'daki noktaların hızı (Hz): 1000 / 500 / 250, usRateHz'e göre. */
uint16_t Imu_PointRateHz( uint16_t usRateHz );

/* Drops a DRDY notification left from before the configuration (a board
 * reset does not reset the IMU). Call right after a successful Imu_Init. */
void Imu_DiscardStaleDrdy( void );

/* Waits for the next DRDY, at most xTimeout (<= APP_IMU_DRDY_TIMEOUT_MAX_MS).
 * pdFAIL on timeout (IMU_DRDY_TIMEOUT counted). */
BaseType_t Imu_WaitDrdy( TickType_t xTimeout );

/* Son çıkış örneğinin FIFO noktalarını SPI2 DMA ile okur, en çok xTimeout
 * (<= APP_IMU_SPI_TIMEOUT_MS). ulTick / ulDrdyCycles = son DRDY'nin tick'i ve
 * DWT sayısı. pdFAIL: süre doldu, DMA hatası ya da kaymış FIFO (IMU_SPI_TIMEOUT
 * sayılır); bir sonraki okuma FIFO'yu boşaltıp yeniden eşitler. */
BaseType_t Imu_ReadBlock( ImuBlock_t * pxOut, TickType_t xTimeout );

#endif /* IMU_SPI_H */
