/* M33 FreeRTOS uygulaması: sup_task (10 ms periyotlu gözetim, LED3, USER2 butonu) ve app_init.
   rx_queue'dan A35 mesajlarını alır, tx_queue'ya SUP/BTN satırı yazar; OpenAMP'ye dokunmaz. */
#include "app.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "supervisor.h"

#include <stdio.h>
#include <string.h>

QueueHandle_t rx_queue;
QueueHandle_t tx_queue;

static uint32_t time_last_cyc;              // app_time_us: son okunan DWT değeri
static uint32_t time_rem_cyc;               // henüz mikrosaniyeye çevrilmemiş çevrim
static uint32_t time_us;                    // birikmiş mikrosaniye (taşar, farklar doğru kalır)
static uint32_t tx_drops;                   // tx_queue dolu: A35'e gidemeyen satır

/* Satırı tx_queue'ya koyar; rpmsg_task yollar. Kuyruk doluysa satır sayılıp atılır. */
static void send_line(const char *text, size_t len)
{
    if ((len == 0U) || (len >= LINK_LINE_MAX))
    {
        return;
    }

    TxLine tx;
    tx.len = (uint8_t)len;
    memcpy(tx.text, text, len);
    if (xQueueSend(tx_queue, &tx, 0) != pdPASS)
    {
        ++tx_drops;                                  // gözetim döngüsü beklememeli
    }
}

uint32_t app_time_us(void)
{
    /* İki task çağırır; okuma-güncelleme birlikte yapılmalı. 400 MHz'de CYCCNT 10,7 s'de taşar,
       sup_task 10 ms'de bir çağırdığı için iki okuma arası hiçbir zaman bu kadar uzamaz. */
    taskENTER_CRITICAL();
    uint32_t cyc = DWT->CYCCNT;
    time_rem_cyc += cyc - time_last_cyc;
    time_last_cyc = cyc;
    uint32_t cyc_per_us = SystemCoreClock / 1000000U;
    time_us += time_rem_cyc / cyc_per_us;
    time_rem_cyc %= cyc_per_us;
    uint32_t now = time_us;
    taskEXIT_CRITICAL();
    return now;
}

/* sup_task: rx_queue -> supervisor; 10 ms'de bir mod, LED, buton ve 500 ms'de bir SUP satırı. */
static void sup_task(void *arg)
{
    (void)arg;

    Supervisor sup;
    sup_init(&sup, app_time_us());
    send_line("RDY freertos 1\n", 15U);

    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        /* Bekleyen bütün mesajlar işlenir; zaman damgası rpmsg_task'ın aldığı an. */
        RxItem item;
        while (xQueueReceive(rx_queue, &item, 0) == pdPASS)
        {
            if (item.bad != 0U)
            {
                sup_on_bad_line(&sup);
            }
            else
            {
                sup_on_msg(&sup, &item.msg, item.rx_us);
            }
        }

        uint32_t now = app_time_us();
        SupMode mode = sup_step(&sup, now);

        if (sup_led_on(mode, now) != 0)
        {
            BSP_LED_On(LED3);
        }
        else
        {
            BSP_LED_Off(LED3);
        }

        /* USER2 aktif düşük: basılıyken pin 0 okunur. */
        int pressed = (BSP_PB_GetState(BUTTON_USER2) == (int32_t)GPIO_PIN_RESET) ? 1 : 0;
        if (sup_button_sample(&sup, pressed) != 0)
        {
            send_line("BTN\n", 4U);                  // A35 hub START/STOP arasında geçiş yapar
        }

        if (sup_report_due(&sup, now) != 0)
        {
            char line[LINK_LINE_MAX];
            size_t len = sup_format_report(&sup, now, line, sizeof(line));
            send_line(line, len);
        }

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));   // sabit periyot: kayma birikmez
    }
}

void app_init(void)
{
    /* DWT çevrim sayacı: mikrosaniye ölçümü için; debugger bağlı olmasa da açılır. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    time_last_cyc = DWT->CYCCNT;

    if (BSP_LED_Init(LED3) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }
    if (BSP_PB_Init(BUTTON_USER2, BUTTON_MODE_GPIO) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }

    /* 16 satır: 50 Hz TM + AT + HB, sup_task 10 ms'de boşaltır; 4 kat pay. */
    rx_queue = xQueueCreate(16U, sizeof(RxItem));
    configASSERT(rx_queue != NULL);
    /* 4 satır: 500 ms'de bir SUP ve ara sıra BTN. */
    tx_queue = xQueueCreate(4U, sizeof(TxLine));
    configASSERT(tx_queue != NULL);

    /* rpmsg_task, sup_task'tan yüksek: A35 satırları gecikmeden zaman damgası alsın. */
    configASSERT(xTaskCreate(app_rpmsg_task, "rpmsg", 1024, NULL, 3, NULL) == pdPASS);  // 1024 word: OpenAMP çağrı derinliği
    configASSERT(xTaskCreate(sup_task, "sup", 512, NULL, 2, NULL) == pdPASS);           // 512 word: snprintf
}
