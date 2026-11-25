#pragma once
#include "frame.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#define MSG_SIZE   23U                               // F746 ham mesajı, bayt
#define MSG_GAP_US 2000U                             // çerçeve içinde bundan uzun sessizlik: çerçeve yarım kaldı

/* F746'dan gelen bir ham mesaj; zamanlar k_cycle_get_32 çevrimi. */
typedef struct
{
    uint8_t data[MSG_SIZE];
    uint32_t first_cycles;                           // T1: çerçevenin ilk baytı alındı
    uint32_t last_cycles;                            // T2: çerçevenin son baytı alındı
} RawMsg;

/* F746'dan gelen bir satır parçası. */
typedef struct
{
    uint8_t size;
    uint8_t data[FRAME_DATA_MAX];                    // sıra baytı + karakterler
} Piece;

extern struct k_msgq raw_queue;                      // kesmeden thread'e RawMsg
extern struct k_msgq piece_queue;                    // kesmeden line thread'ine Piece
extern volatile uint32_t piece_drops;                // kuyruk doluyken gelen parça
extern volatile uint32_t rx_errors;                  // hat hatası
extern volatile uint32_t bad_len;                    // bozuk ya da yarım kalan çerçeve
extern volatile uint32_t queue_drops;                // kuyruk doluyken gelen mesaj

void uart_isr(const struct device *dev, void *user_data); // USART6 kesmesi
