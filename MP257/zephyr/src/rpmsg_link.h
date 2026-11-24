#pragma once
#include <zephyr/kernel.h>

#define LINK_LEASE_MS 3000U                          // son '1'den bu kadar sonra gönderme kendiliğinden durur

typedef enum
{
    LINK_WAIT,                                       // Linux'un kanalı açması bekleniyor
    LINK_OPEN,                                       // uç nokta açık, /dev/rpmsg0 oluşur
    LINK_FAILED                                      // kanal kurulamadı, neden link_error'da
} LinkState;

extern volatile uint32_t link_state;                 // LinkState
extern volatile int32_t link_error;                  // kanalı durduran hata kodu
extern volatile uint32_t link_rx;                    // Linux'tan gelen mesaj
extern volatile uint32_t link_unknown;               // tanınmayan komut
extern volatile uint32_t link_on;                    // Linux paket istiyor: '1' açar, '0' ya da sürenin dolması kapatır
extern volatile uint32_t link_expired;               // '1' yenilenmediği için durma sayısı
extern volatile uint32_t link_sent;                  // Linux'a verilen paket
extern volatile uint32_t link_skipped;               // Linux istemediği için atılan paket
extern volatile uint32_t link_drops;                 // tampon dolu olduğu için atılan paket

void lease_isr(struct k_timer *timer);               // süre doldu: göndermeyi kapatır
int link_send(const void *data, size_t size);        // paketi Linux'a verir, beklemez; başarı 1
