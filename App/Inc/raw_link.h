#pragma once
#include <stdint.h>

void raw_link_init(void);                                                        // USART6'yı APP_RAW_BAUD hızına alır
void raw_link_send(const uint8_t *payload, uint16_t seq, uint32_t drdy_cycles);  // APP_LEN_TELEM bayt yollar, beklemez
uint32_t raw_link_last(void);                                                    // son giden: SEQ << 16 | DRDY -> son bit us; 0 = yok
void raw_link_tx_done_isr(void);                                                 // USART6 TX bitti kesmesi
