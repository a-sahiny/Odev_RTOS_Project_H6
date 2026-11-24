#pragma once
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#define MSG_SIZE   23U                               // F746 ham mesajı, bayt
#define MSG_GAP_US 2000U                             // bundan uzun sessizlik mesaj sınırıdır (mesaj 0,5 ms, ara 9,5 ms)

/* F746'dan gelen bir ham mesaj; zamanlar k_cycle_get_32 çevrimi. */
typedef struct
{
    uint8_t data[MSG_SIZE];
    uint32_t first_cycles;                           // T1: ilk bayt alındı
    uint32_t last_cycles;                            // T2: 23. bayt alındı
} RawMsg;

extern struct k_msgq raw_queue;                      // kesmeden thread'e RawMsg
extern volatile uint32_t rx_errors;                  // hat hatası
extern volatile uint32_t bad_len;                    // 23 bayt olmayan mesaj
extern volatile uint32_t queue_drops;                // kuyruk doluyken gelen mesaj

void uart_isr(const struct device *dev, void *user_data); // USART6 kesmesi
