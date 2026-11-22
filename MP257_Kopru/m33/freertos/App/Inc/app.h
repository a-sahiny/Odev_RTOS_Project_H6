/* M33 FreeRTOS uygulaması: rpmsg_task (OpenAMP sahibi) ve sup_task (gözetim, LED, buton) arasındaki ortak tipler. */
#pragma once
#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "link_proto.h"

typedef struct
{
    uint8_t bad;                            // 1: satır ayrıştırılamadı, msg geçersiz
    uint32_t rx_us;                         // satırın rpmsg_task'a geldiği an
    LinkMsg msg;
} RxItem;

typedef struct
{
    uint8_t len;                            // '\n' dahil karakter
    char text[LINK_LINE_MAX];
} TxLine;

extern QueueHandle_t rx_queue;              // rpmsg_task -> sup_task
extern QueueHandle_t tx_queue;              // sup_task -> rpmsg_task

void app_init(void);                        // kuyrukları ve task'ları kurar; MX_FREERTOS_Init içinden çağrılır
uint32_t app_time_us(void);                 // DWT tabanlı mikrosaniye sayacı (uint32, taşar)
void app_rpmsg_task(void *arg);             // app_rpmsg.c: OpenAMP + VIRT_UART0 sahibi
