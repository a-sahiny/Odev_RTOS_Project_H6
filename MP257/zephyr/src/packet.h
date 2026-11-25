#pragma once
#include "app.h"

#define PKT_HEADER_SIZE 9U                           // SOF'tan LEN'e
#define M33_US_SIZE     2U                           // önceki paketin T5 - T1 süresi, µs, yüksek bayt önce
#define PKT_SIZE        (PKT_HEADER_SIZE + MSG_SIZE + M33_US_SIZE + 1U) // başlık + ham mesaj + M33 süresi + CRC8 = 35

#define RAW_FLAGS_OFFSET    18U                      // ham mesajda flags
#define RAW_PREV_SEQ_OFFSET 19U                      // önceki mesajın sırası, yüksek bayt önce
#define RAW_FLAG_PREV       0x04U                    // prev_seq geçerli

uint16_t packet_seq(const uint8_t *raw);                    // paketin SEQ'i: prev_seq + 1, prev yoksa 0
void packet_build(uint8_t *out, const uint8_t *raw, uint16_t m33_us); // 23 ham bayt + M33 süresi: 35 baytlık TELEM paketi
