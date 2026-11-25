#pragma once
#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"

void raw_link_init(void);                                                        // USART6'yı APP_RAW_BAUD hızına alır
void raw_link_send(const uint8_t *payload, uint16_t seq, uint32_t drdy_cycles);  // APP_LEN_TELEM bayt yollar, beklemez
int raw_link_send_line(const uint8_t *data, size_t size, TickType_t wait);        // satırı yollar, bitmesini bekler; başarı 1
uint32_t raw_link_last(void);                                                    // son giden: SEQ << 16 | DRDY -> son bit us; 0 = yok
void raw_link_tx_done_isr(void);                                                 // USART6 TX bitti kesmesi
void raw_link_rx_isr(uint16_t size);                                              // USART6 DMA'sı 'size' konumuna geldi
void raw_link_error_isr(void);                                                   // USART6 hat hatası: almayı yeniden kurar
int raw_link_linux_on(void);                                                     // MP257'de Linux okuyor: 1
