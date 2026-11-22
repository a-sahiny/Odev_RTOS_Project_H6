/**
  ******************************************************************************
  * @file    net_app.h
  * @brief   FreeRTOS-Plus-TCP start-up: VNET interface over USART1 (and,
  *          with NET_USE_ETH 1, the board's Ethernet port).
  ******************************************************************************
  */
#ifndef NET_APP_H
#define NET_APP_H

#include "FreeRTOS.h"

/* DWT çevrim sayacını açar, PRNG'yi cihaz kimliği ve donanım RNG'siyle
 * tohumlar. App_Init'te SysState_Init'ten (EPOCH) önce bir kez çağrılır. */
void NetApp_RandomInit(void);

/* Creates the network interface(s) + end-point(s) and starts the IP task.
 * Call once, before vTaskStartScheduler(), after SysState_Init (the network
 * event hook writes NET_READY). Returns pdPASS on success. The network state
 * is the NET_READY bit (SysState_Get / SysState_WaitNetReady). */
BaseType_t NetApp_Init(void);

#endif /* NET_APP_H */
