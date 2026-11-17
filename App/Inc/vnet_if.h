/**
  ******************************************************************************
  * @file    vnet_if.h
  * @brief   Virtual Ethernet interface over USART1 (VNET) for FreeRTOS-Plus-TCP,
  *          and the vnetRx task that turns host lines into frames and console
  *          commands (docs 3.1, 10.1, 13, 14).
  ******************************************************************************
  */
#ifndef VNET_IF_H
#define VNET_IF_H

#include "FreeRTOS.h"

/* = NetworkInterface_t (FreeRTOS_Routing.h); declared by tag so this header
 * does not pull in the TCP/IP stack. */
struct xNetworkInterface;

/* NetApp_Init, before FreeRTOS_IPInit_Multi. Fills pxInterface with the
 * VNET functions and adds it to the stack (as pxSTM32_FillInterfaceDescriptor
 * does). xIndex is only kept for the same signature. Returns pxInterface. */
struct xNetworkInterface * VNet_FillInterfaceDescriptor( BaseType_t xIndex, struct xNetworkInterface * pxInterface );

/* App_Init (for now main.c), before the scheduler. Creates the static
 * vnetRx task. A second call does nothing. Always pdPASS. */
BaseType_t VNet_CreateRxTask( void );

#endif /* VNET_IF_H */
