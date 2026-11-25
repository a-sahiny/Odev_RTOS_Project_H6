#pragma once
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>

#define LINUX_MSG_SIZE 4U                            // F746'ya giden "Linux var / yok" mesajı, bayt
#define CMD_CHARS      31U                           // bir komut çerçevesindeki PC baytı
#define CMD_FRAME_MAX  (4U + 1U + CMD_CHARS)         // SOF, tür, uzunluk, sıra, veri, CRC8

void linux_msg_build(uint8_t *out, int on);                  // LINUX_MSG_SIZE bayt kurar
size_t cmd_frame_build(uint8_t *out, uint8_t seq, size_t n); // out[4]'teki n baytı çerçeveler; çerçeve uzunluğu
void f746_tx_state(int on);                                  // durum mesajını kesmeyle yollar, beklemez
void f746_tx_command(const uint8_t *data, size_t size);      // PC'nin baytlarını sıraya koyar, beklemez
void f746_tx_isr(const struct device *dev);                  // USART6 yazmaya hazır
