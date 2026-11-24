#pragma once
#include "app.h"

#define PKT_HEADER_SIZE 9U                           // SOF'tan LEN'e
#define PKT_SIZE        (PKT_HEADER_SIZE + MSG_SIZE + 1U) // başlık + ham mesaj + CRC8 = 33

#define RAW_FLAGS_OFFSET    18U                      // ham mesajda flags
#define RAW_PREV_SEQ_OFFSET 19U                      // önceki mesajın sırası, yüksek bayt önce
#define RAW_FLAG_PREV       0x04U                    // prev_seq geçerli

uint16_t packet_seq(const uint8_t *raw);                    // paketin SEQ'i: prev_seq + 1, prev yoksa 0
void packet_build(uint8_t *out, const uint8_t *raw);        // 23 ham bayttan 33 baytlık TELEM paketi
