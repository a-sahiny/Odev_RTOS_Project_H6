/* MP257 Cortex-M33 uygulaması. M6: F746'nın ham mesajlarını TELEM paketine çevirip Linux'a verir, sırasını ve sürelerini tutar. */
#include "packet.h"
#include "rpmsg_link.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>

#define LED_MSGS 50U                                 // 100 Hz'de LED 0,5 saniyede bir değişir

/* Bir sürenin özeti, k_cycle_get_32 çevrimi; jitter = max - min. */
typedef struct
{
    uint32_t count;
    uint32_t min;
    uint32_t max;
    uint64_t sum;
} Stat;

// Kesme ile paylaşılan nesneler; 8 mesaj = 80 ms pay
K_MSGQ_DEFINE(raw_queue, sizeof(RawMsg), 8, 4);
volatile uint32_t rx_errors;
volatile uint32_t bad_len;
volatile uint32_t queue_drops;

// Linux'tan okunan sonuçlar
volatile uint32_t msg_count;                         // alınan tam mesaj
volatile uint32_t seq_gaps;                          // sırada atlama
volatile Stat rx_time = { .min = UINT32_MAX };       // T2 - T1: ilk bayttan 23. bayta
volatile Stat wake_time = { .min = UINT32_MAX };     // T3 - T2: kesmeden thread'e
volatile Stat proc_time = { .min = UINT32_MAX };     // T4 - T3: sıra denetimi ve paket kurma
volatile Stat send_time = { .min = UINT32_MAX };     // T5 - T4: paketi Linux'a verme, yalnız gönderilenler
uint8_t last_packet[PKT_SIZE];                       // son TELEM paketi

// Donanım tanımındaki led0: LD9, GPIOH pin 6
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

// PF13 TX, PF14 RX, 460800
static const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(usart6));

/* Bir süreyi özete ekler. */
static void stat_add(volatile Stat *stat, uint32_t cycles)
{
    if (cycles < stat->min)
    {
        stat->min = cycles;
    }

    if (cycles > stat->max)
    {
        stat->max = cycles;
    }

    stat->sum += cycles;
    ++stat->count;
}

/* prev_seq bir öncekinden 1 fazla değilse atlama sayar. */
static void check_seq(const RawMsg *msg)
{
    static uint16_t last_seq;
    static int have_seq;

    // F746 yeni açıldı: henüz önceki mesaj yok, sıra yeniden başlar
    if (!(msg->data[RAW_FLAGS_OFFSET] & RAW_FLAG_PREV))
    {
        have_seq = 0;
        return;
    }

    uint16_t seq = sys_get_be16(&msg->data[RAW_PREV_SEQ_OFFSET]);

    if (have_seq && (uint16_t)(seq - last_seq) != 1U)
    {
        ++seq_gaps;
    }

    last_seq = seq;
    have_seq = 1;
}

/* LED ve USART6'yı hazırlar, gelen her mesajı işler; dönmez. */
int main(void)
{
    if (!gpio_is_ready_dt(&led) || !device_is_ready(uart))
    {
        k_panic();                                   // sürücü açılmadı
    }

    if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE) != 0)
    {
        k_panic();                                   // pin çıkış yapılamadı
    }

    if (uart_irq_callback_user_data_set(uart, uart_isr, NULL) != 0)
    {
        k_panic();                                   // kesme bağlanamadı
    }

    (void)uart_err_check(uart);                      // açılıştan kalan hata bayrakları temizlenir
    uart_irq_rx_enable(uart);
    uart_irq_err_enable(uart);

    for (;;)
    {
        RawMsg msg;

        k_msgq_get(&raw_queue, &msg, K_FOREVER);     // thread mesaj gelene kadar uyur
        uint32_t woken = k_cycle_get_32();           // T3

        check_seq(&msg);
        packet_build(last_packet, msg.data);
        uint32_t built = k_cycle_get_32();           // T4

        int sent = link_send(last_packet, PKT_SIZE);
        uint32_t handed = k_cycle_get_32();          // T5

        // Özetler T5'ten sonra: ölçüm kendi süresini aşamalara eklemez
        stat_add(&rx_time, msg.last_cycles - msg.first_cycles);
        stat_add(&wake_time, woken - msg.last_cycles);
        stat_add(&proc_time, built - woken);

        if (sent)
        {
            stat_add(&send_time, handed - built);
        }

        ++msg_count;                                 // paket hazır olduktan sonra artar

        if (msg_count % LED_MSGS == 0 && gpio_pin_toggle_dt(&led) != 0)
        {
            k_panic();                               // pin yazılamadı
        }
    }
}
