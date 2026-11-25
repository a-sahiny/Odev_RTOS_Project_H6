/* PC testi için FreeRTOS, HAL ve Zephyr taklidi; yalnız raw_link.c ile f746_tx.c'nin kullandıkları. */
#pragma once
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define APP_TELEM_TARGET_MP257 1
#define APP_TELEM_TARGET 1
#define APP_LEN_TELEM 23U
#define APP_RAW_BAUD 460800U
#define configCPU_CLOCK_HZ 216000000UL
#define configASSERT(x) assert(x)
#define taskENTER_CRITICAL()
#define taskEXIT_CRITICAL()
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef int TaskHandle_t;
#define pdTRUE 1
#define pdFALSE 0
#define portYIELD_FROM_ISR(w) ((void)(w))
#define xTaskGetCurrentTaskHandle() 1
#define vTaskNotifyGiveFromISR(t, w) ((void)(t), (void)(w), notified = 1)
extern int notified;
uint32_t ulTaskNotifyTake(int clear, uint32_t wait);

typedef enum { HAL_OK, HAL_BUSY } HAL_StatusTypeDef;
#define HAL_UART_STATE_READY 0x20U
#define HAL_UART_STATE_BUSY_TX 0x21U
typedef struct { struct { uint32_t BaudRate; } Init; uint32_t gState, RxState; } UART_HandleTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
#define GPIO_PIN_7 0x80U
#define GPIO_MODE_AF_PP 2U
#define GPIO_PULLUP 1U
#define GPIO_SPEED_FREQ_VERY_HIGH 3U
#define GPIO_AF8_USART6 8U
#define GPIOC 0
extern uint8_t *rx_target;
extern uint32_t tick_ms;
static struct { uint32_t CYCCNT; } dwt;
#define DWT (&dwt)
#define HAL_UART_Init(u) ((void)(u), HAL_OK)
#define HAL_GPIO_Init(p, g) ((void)(g))
#define HAL_UARTEx_ReceiveToIdle_DMA(u, b, n) (rx_target = (b), HAL_OK)
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, const uint8_t *data, uint16_t size);
#define HAL_UART_AbortTransmit(u) ((u)->gState = HAL_UART_STATE_READY, HAL_OK)
#define HAL_GetTick() tick_ms

struct device { int unused; };
static const struct device usart6_dev;
#define DT_NODELABEL(x) x
#define DEVICE_DT_GET(x) (&usart6_dev)
extern int irq_on, fifo_room, wire_len;
extern uint8_t wire[16384];
#define uart_irq_tx_enable(d) (irq_on = 1)
#define uart_irq_tx_disable(d) ((void)(d), irq_on = 0)

/* FIFO'ya en çok fifo_room bayt alır. */
static int uart_fifo_fill(const struct device *dev, const uint8_t *data, int size)
{
    (void)dev;
    int n = size < fifo_room ? size : fifo_room;

    for (int i = 0; i < n; ++i)
    {
        wire[wire_len++] = data[i];
    }

    return n;
}

/* Zephyr crc8: yansıtmasız, MSB önce. */
static inline uint8_t crc8(const uint8_t *data, size_t size, uint8_t poly, uint8_t crc, bool reversed)
{
    (void)reversed;

    for (size_t i = 0; i < size; ++i)
    {
        crc ^= data[i];

        for (int b = 0; b < 8; ++b)
        {
            crc = (uint8_t)(crc & 0x80U ? (crc << 1) ^ poly : crc << 1);
        }
    }

    return crc;
}

void UartLink_FeedFromISR(const uint8_t *data, size_t size, BaseType_t lost, BaseType_t *woken);

/* Zephyr ring_buf: tek yazar, tek okur. */
struct ring_buf
{
    uint8_t *buf;
    uint32_t size, head, tail;
};
#define RING_BUF_DECLARE(name, n) static uint8_t name##_data[n]; static struct ring_buf name = { name##_data, n, 0, 0 }

static inline uint32_t ring_buf_space_get(struct ring_buf *r)
{
    return r->size - (r->head - r->tail);
}

static inline uint32_t ring_buf_put(struct ring_buf *r, const uint8_t *data, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        r->buf[r->head++ % r->size] = data[i];
    }

    return n;
}

static inline uint32_t ring_buf_get(struct ring_buf *r, uint8_t *data, uint32_t n)
{
    uint32_t i = 0;

    for (; i < n && r->tail != r->head; ++i)
    {
        data[i] = r->buf[r->tail++ % r->size];
    }

    return i;
}
