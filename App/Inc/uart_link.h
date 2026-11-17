/**
  ******************************************************************************
  * @file    uart_link.h
  * @brief   USART1, the only physical link to the PC (docs 13.1). Moves bytes
  *          only: RX DMA -> sbUartRx -> vnetRx, uartTx -> TX DMA, and a
  *          blocking write for before the scheduler / fault time.
  ******************************************************************************
  * uart_link.c is the only file that touches huart1, its two DMA streams and
  * the RX DMA buffer.
  *   sbUartRx   writer: USART1 / DMA2 Stream2 ISR (NVIC 5); reader: vnetRx
  *   TX done    task notification (index 0) of uartTx, given by the ISR
  */
#ifndef UART_LINK_H
#define UART_LINK_H

#include "FreeRTOS.h"
#include "stream_buffer.h"

/* Creates sbUartRx (static) and resets the RX bookkeeping. Call once from
 * App_Init, before the scheduler starts; a second call changes nothing.
 * Checks that CubeMX linked a circular RX DMA to huart1. */
void UartLink_Init( void );

/* Starts the circular DMA reception with IDLE detection. Called by the VNET
 * pfInitialise (IP task), possibly more than once: if the reception is
 * already running it does nothing and returns pdPASS. pdFAIL if the HAL
 * refuses (the stack calls pfInitialise again later). Never waits; masks
 * NVIC 5..15 for the few microseconds of the HAL start. From the first
 * success on, the UART error callback restarts the reception by itself
 * whenever the HAL stops it (UART or DMA error). */
BaseType_t UartLink_StartRx( void );

/* The RX StreamBuffer (sbUartRx). vnetRx, its ONE reader, calls
 * xStreamBufferReceive on it directly (report 092); the kernel wakes it
 * with its task notification index 0. */
StreamBufferHandle_t UartLink_GetRxStream( void );

/* Number of RX loss events so far (sbUartRx full, DMA lap, UART error).
 * It only grows; when it changed while a line was being collected, bytes
 * are missing from that line (line_rx drops it). Any task context. */
uint32_t UartLink_GetLossCount( void );

/* uartTx only. Sends uxLen bytes (1..APP_UART_TX_DMA_BYTES) of pucData with
 * the TX DMA and blocks until the last bit has left the line, at most xWait
 * ticks (line time + APP_UART_TX_MARGIN_MS, <= 29 ms at 625000). pucData must stay
 * untouched until the call returns. pdPASS when sent; pdFAIL when the HAL
 * refused or the wait timed out (the transfer is then aborted and
 * eStatTxUartStall counted). The wait uses uartTx's task notification
 * index 0. */
BaseType_t UartLink_SendDma( const uint8_t * pucData, size_t uxLen, TickType_t xWait );

/* DWT->CYCCNT taken in the TX done interrupt of the last line (USART TC:
 * its last bit has left the line). uartTx reads it after UartLink_SendDma
 * returned pdPASS: then it belongs to that line (report 091). */
uint32_t UartLink_GetTxDoneCycles( void );

/* App_Halt only (report 092): one TX DMA transfer of pucData, started and
 * left running; returns at once (no polling, no interrupt needed). A DMA
 * line still being sent is aborted first. pucData must stay valid. */
void UartLink_HaltWrite( const uint8_t * pucData, size_t uxLen );

#endif /* UART_LINK_H */
