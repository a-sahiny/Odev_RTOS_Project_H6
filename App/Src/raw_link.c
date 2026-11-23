/* MP257'ye giden ham telemetri hattı (USART6). meas task'ı yollar, TX bitti kesmesi
   DRDY -> son bit süresini ölçer. Yalnız APP_TELEM_TARGET_MP257 derlemesinde vardır. */
#include "raw_link.h"
#include "app_config.h"

#if (APP_TELEM_TARGET == APP_TELEM_TARGET_MP257)

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart6;

static uint8_t tx_buf[APP_LEN_TELEM];   // DMA okur; D-cache kapalı
static uint16_t tx_seq;                 // hattaki mesajın SEQ'i
static uint32_t tx_drdy;                // hattaki mesajın DRDY anı (DWT çevrimi)
static volatile uint32_t last_sent;     // kesme yazar, meas okur
static volatile uint32_t tx_drops;      // hat meşgulken gelen ya da başlatılamayan mesaj (debugger'dan okunur)

/* USART6'yı APP_RAW_BAUD hızına alır. CubeMX 230400 ile kurdu, pin ve DMA ayarı ondan kalır. */
void raw_link_init(void)
{
    huart6.Init.BaudRate = APP_RAW_BAUD;
    configASSERT(HAL_UART_Init(&huart6) == HAL_OK);
}

/* Bir mesajı DMA ile başlatır, beklemez. Hat meşgulse mesaj düşer ve sayılır. */
void raw_link_send(const uint8_t *payload, uint16_t seq, uint32_t drdy_cycles)
{
    if (huart6.gState != HAL_UART_STATE_READY)
    {
        ++tx_drops;                     // DMA hâlâ tx_buf'ı okuyor, üstüne yazılmaz
        return;
    }

    memcpy(tx_buf, payload, sizeof(tx_buf));
    tx_seq = seq;                       // DMA'dan önce: TX bitti kesmesi bunları okur
    tx_drdy = drdy_cycles;

    if (HAL_UART_Transmit_DMA(&huart6, tx_buf, sizeof(tx_buf)) != HAL_OK)
    {
        ++tx_drops;
    }
}

/* Son giden mesajın SEQ'i ve DRDY -> son bit süresi; henüz giden yoksa 0. */
uint32_t raw_link_last(void)
{
    return last_sent;
}

/* USART6 TX bitti: son bit hatta çıktı, süreyi kaydeder. */
void raw_link_tx_done_isr(void)
{
    uint32_t us = (DWT->CYCCNT - tx_drdy) / (configCPU_CLOCK_HZ / 1000000UL);    // işaretsiz fark CYCCNT taşmasında da doğru

    if (us > 0xFFFFU)
    {
        us = 0xFFFFU;
    }

    last_sent = ((uint32_t)tx_seq << 16) | us;
}

#endif
