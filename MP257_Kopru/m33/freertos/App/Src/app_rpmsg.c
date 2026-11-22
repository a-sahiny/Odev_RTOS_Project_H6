/* rpmsg_task: OpenAMP'nin ve /dev/ttyRPMSG0 kanalının (VIRT_UART0) tek sahibi.
   Gelen baytları satıra çevirip rx_queue ile sup_task'a verir; tx_queue'daki satırları A35'e yollar. */
#include "app.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "openamp.h"
#include "virt_uart.h"

extern IPCC_HandleTypeDef hipcc1;           // main.c: MX_IPCC_Init kurar (CoproSync de kullanır)

static VIRT_UART_HandleTypeDef vuart0;
static LineRx line_rx;
static uint32_t rx_drops;                   // rx_queue dolu: kaybolan satır (yalnız bu task yazar)

/* VIRT_UART0 RX: OPENAMP_check_for_message içinden, yani bu task'ın bağlamında çağrılır (ISR değil). */
static void vuart0_rx(VIRT_UART_HandleTypeDef *huart)
{
    uint32_t now = app_time_us();
    for (uint16_t i = 0U; i < huart->RxXferSize; ++i)
    {
        const char *line = NULL;
        if (line_rx_push(&line_rx, (char)huart->pRxBuffPtr[i], &line) == 0)
        {
            continue;
        }

        RxItem item;
        item.rx_us = now;
        item.bad = (link_parse(line, &item.msg) == 0) ? 1U : 0U;
        if (xQueueSend(rx_queue, &item, 0) != pdPASS)
        {
            ++rx_drops;                              // beklemek OpenAMP'yi tıkar; kaybı say
        }
    }
}

/* rpmsg_task: OpenAMP'yi kurar, 1 ms'de bir mesaj yoklar, bekleyen TX satırlarını yollar. */
void app_rpmsg_task(void *arg)
{
    (void)arg;

    /* OpenAMP: kaynak tablosu ve vring'ler A35 remoteproc'un ayırdığı paylaşımlı bellekte. */
    if (MX_OPENAMP_Init(RPMSG_REMOTE, &hipcc1) != 0)
    {
        Error_Handler();
    }
    if (MX_OPENAMP_RPMsg_Init(RPMSG_REMOTE, NULL, &hipcc1) != 0)
    {
        Error_Handler();
    }

    /* "rpmsg-tty" kanalı: Linux'ta rpmsg_tty sürücüsü /dev/ttyRPMSG0 açar. */
    if (VIRT_UART_Init(&vuart0) != VIRT_UART_OK)
    {
        Error_Handler();
    }
    if (VIRT_UART_RegisterCallback(&vuart0, VIRT_UART_RXCPLT_CB_ID, vuart0_rx) != VIRT_UART_OK)
    {
        Error_Handler();
    }
    line_rx_init(&line_rx);

    for (;;)
    {
        OPENAMP_check_for_message();                 // RX callback'leri burada çalışır

        TxLine tx;
        while (xQueueReceive(tx_queue, &tx, 0) == pdPASS)
        {
            /* A35 kanalı açmadan (cat /dev/ttyRPMSG0 yokken) gönderim hata verebilir; satır atılır. */
            (void)VIRT_UART_Transmit(&vuart0, tx.text, tx.len);
        }

        vTaskDelay(pdMS_TO_TICKS(1));                // yoklama: IPCC kesmesi bayrak kurar, gecikme <= 1 ms
    }
}
