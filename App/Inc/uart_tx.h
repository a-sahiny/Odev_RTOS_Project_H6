/**
  ******************************************************************************
  * @file    uart_tx.h
  * @brief   The single TX worker (uartTx): frames and JSON reports to USART1
  *          (docs 3.2, 5.2, 10.1, 13.1).
  ******************************************************************************
  * Nothing else writes to the UART once the scheduler runs. Producers put an
  * item in xUartTxQueue without waiting; uartTx turns it into one line,
  * gives the network buffer / report block back before the line is sent and
  * sends the line with UartLink_SendDma. The last APP_TXQ_RESERVE slots of
  * the queue are for control frames only (command replies).
  */
#ifndef UART_TX_H
#define UART_TX_H

#include <stdint.h>

#include "FreeRTOS.h"

#include "app_types.h"

/* = NetworkBufferDescriptor_t (FreeRTOS_IP.h); declared by tag so this
 * header does not pull in the TCP/IP stack. */
struct xNETWORK_BUFFER;

/* App_Init, before the scheduler. Creates the static TX queue (and, from
 * step 10d, the uartTx task). A second call does nothing. Always pdPASS:
 * static objects cannot fail to be created. */
BaseType_t UartTx_Init( void );

/* IP task only (pfOutput of the virtual NIC). Classifies the frame and puts
 * it in xUartTxQueue without waiting. pdPASS: the buffer now belongs to
 * uartTx. pdFAIL (bad length, queue full, or only the control reserve left
 * for a non-control frame): the buffer stays with the caller, who releases
 * it. Counted either way (L7). */
BaseType_t UartTx_PostFrame( struct xNETWORK_BUFFER * pxDesc );

/* Hands a finished report block to uartTx (xUartTxQueue, wait 0). pdPASS:
 * the block now belongs to uartTx, which releases it after copying the line.
 * pdFAIL (queue full or only the control reserve left): the block stays with
 * the caller, who releases it. Task context only. */
BaseType_t UartTx_PostReport( ReportDesc_t * pxReport );

/* TXDELAY injection (docs 13.5, T06): uartTx waits usMs before every item
 * until the next call; 0 switches it off. usMs <= APP_TXDELAY_MAX_MS
 * (line_rx rejects more; here it asserts, report 062 B8). Called by the
 * console (vnetRx). Never waits. */
void UartTx_SetInjectedDelay( uint16_t usMs );

/* The TXDELAY in force, in ms (0 = off). The only copy of it (report 062,
 * B10): the console reads it for the "inj" report field. Any context. */
uint16_t UartTx_GetInjectedDelay( void );

/* The last TELEM whose line left the UART, as SEQ << 16 | its DRDY -> TX
 * done time in us (limited to 65535); 0 before the first. Written by uartTx,
 * read by meas for the next TELEM; one 32-bit atomic word. */
uint32_t UartTx_GetLastTelem( void );

#endif /* UART_TX_H */
