/**
  ******************************************************************************
  * @file    FreeRTOSIPConfig.h
  * @brief   FreeRTOS-Plus-TCP V4.4.1 configuration for STM32F746G-DISCO:
  *          VNET over USART1 (vnet_if.c); the STM32 ETH driver stays linked
  *          for the buffer RAM (BufferAllocation_1) and NET_USE_ETH.
  ******************************************************************************
  * Anything not set here takes its default from FreeRTOSIPConfigDefaults.h.
  */
#ifndef FREERTOS_IP_CONFIG_H
#define FREERTOS_IP_CONFIG_H

/* NetworkInterface/STM32 selects the HAL header from the family macro before
 * any ST header is included; stm32f7xx.h defines the same macro (guarded). */
#ifndef STM32F7
    #define STM32F7
#endif

/*----------------------------------------------------------------------------
 * General
 *--------------------------------------------------------------------------*/
#define ipconfigBYTE_ORDER                          pdFREERTOS_LITTLE_ENDIAN
#define ipconfigUSE_IPv4                            1
#define ipconfigUSE_IPv6                            0
#define ipconfigIPv4_BACKWARD_COMPATIBLE            0
#define ipconfigCOMPATIBLE_WITH_SINGLE              0

/* IP task: below the EMAC task (configMAX_PRIORITIES - 1). */
#define ipconfigIP_TASK_PRIORITY                    ( configMAX_PRIORITIES - 2 )
#define ipconfigIP_TASK_STACK_SIZE_WORDS            ( configMINIMAL_STACK_SIZE * 5 )
#define ipconfigEVENT_QUEUE_LENGTH                  ( ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS + 5 )
#define ipconfigUSE_NETWORK_EVENT_HOOK              1
#define ipconfigCHECK_IP_QUEUE_SPACE                1   /* uxGetMinimumIPQueueSpace (app_stats) */

/*----------------------------------------------------------------------------
 * Driver (portable/NetworkInterface/STM32) requirements
 *--------------------------------------------------------------------------*/
#define ipconfigZERO_COPY_RX_DRIVER                 1
#define ipconfigZERO_COPY_TX_DRIVER                 1
#define ipconfigUSE_LINKED_RX_MESSAGES              1
#define ipconfigDRIVER_INCLUDED_RX_IP_CHECKSUM      1   /* VNET checks them and counts the reason (vnet_if.c) */
#define ipconfigDRIVER_INCLUDED_TX_IP_CHECKSUM      0   /* VNET has no MAC: the stack fills them, the host checks */
#define ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES 1
/* ETH_RX_BUF_SIZE (1536, CubeMX) == driver buffer size is checked in net_app.c */
#define ipconfigPORT_SUPPRESS_WARNING               1

/*----------------------------------------------------------------------------
 * Buffers (static, allocated by the driver in section .EthBuffersSection)
 *--------------------------------------------------------------------------*/
#define ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS      18      /* report 086: TX queue 6 -> 8, 4 spare kept */
#define ipconfigNETWORK_MTU                         1500
#define ipconfigTCP_MSS                             1460

/*----------------------------------------------------------------------------
 * Protocols
 *--------------------------------------------------------------------------*/
/* UDP only, static address (D8): no TCP, DHCP or DNS. */
#define ipconfigUSE_TCP                             0
#define ipconfigUSE_DHCP                            0
#define ipconfigUSE_DHCP_HOOK                       0
#define ipconfigDHCP_REGISTER_HOSTNAME              0
#define ipconfigUSE_DNS                             0
#define ipconfigUSE_LLMNR                           0
#define ipconfigUSE_NBNS                            0
#define ipconfigUSE_MDNS                            0
#define ipconfigREPLY_TO_INCOMING_PINGS             1
#define ipconfigSUPPORT_OUTGOING_PINGS              0
#define ipconfigSUPPORT_SELECT_FUNCTION             0
#define ipconfigARP_CACHE_ENTRIES                   8

/*----------------------------------------------------------------------------
 * Sockets
 *--------------------------------------------------------------------------*/
#define ipconfigSOCK_DEFAULT_RECEIVE_BLOCK_TIME     pdMS_TO_TICKS( 5000 )
#define ipconfigSOCK_DEFAULT_SEND_BLOCK_TIME        pdMS_TO_TICKS( 5000 )
#define ipconfigALLOW_SOCKET_SEND_WITHOUT_BIND      1
/* UDP receive callbacks (udp_in.c, D3): a datagram is consumed in the IP task
 * and never queued on the socket. The limit of 1 is the safety net if a
 * callback is not installed (docs 9). */
#define ipconfigUSE_CALLBACKS                       1
#define ipconfigUDP_MAX_RX_PACKETS                  1

/*----------------------------------------------------------------------------
 * Logging (off; route FreeRTOS_printf to a UART to enable)
 *--------------------------------------------------------------------------*/
#define ipconfigHAS_DEBUG_PRINTF                    0
#define ipconfigHAS_PRINTF                          0

/*----------------------------------------------------------------------------
 * Trace hooks -> app_stats counters (D22). Must stay the last line: it is
 * read before FreeRTOSIPConfigDefaults.h / IPTraceMacroDefaults.h.
 *--------------------------------------------------------------------------*/
#include "app_trace_hooks.h"

#endif /* FREERTOS_IP_CONFIG_H */
