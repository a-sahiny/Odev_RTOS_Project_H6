#pragma once
#include <zephyr/kernel.h>

typedef enum
{
    LINK_WAIT,                                       // Linux'un kanalı açması bekleniyor
    LINK_OPEN,                                       // uç nokta açık, /dev/rpmsg0 oluşur
    LINK_FAILED                                      // kanal kurulamadı, neden link_error'da
} LinkState;

extern volatile uint32_t link_state;                 // LinkState
extern volatile int32_t link_error;                  // kanalı durduran hata kodu
extern volatile uint32_t link_rx;                    // Linux'tan gelen mesaj
