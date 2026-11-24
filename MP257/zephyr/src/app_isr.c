/* Kesmeler: USART6 baytlarını mesaja ayırıp main thread'ine, IPCC haberini link thread'ine verir. */
#include "app.h"
#include "ipcc.h"
#include <zephyr/drivers/uart.h>

static RawMsg msg;                                   // toplanan mesaj
static uint32_t msg_len;                             // toplanan bayt sayısı
static uint32_t last_byte_cycles;                    // son baytın alındığı an

/* Bir baytı mesaja ekler; 23. baytta mesajı kuyruğa koyar. */
static void add_byte(uint8_t byte)
{
    uint32_t now = k_cycle_get_32();

    // Başlık yok: mesaj sınırını sessizlik gösterir; yarım kalan mesaj atılır
    if (msg_len > 0 && now - last_byte_cycles > k_us_to_cyc_floor32(MSG_GAP_US))
    {
        ++bad_len;
        msg_len = 0;
    }

    last_byte_cycles = now;

    if (msg_len == 0)
    {
        msg.first_cycles = now;                      // T1
    }

    msg.data[msg_len] = byte;
    ++msg_len;

    if (msg_len == MSG_SIZE)
    {
        msg.last_cycles = now;                       // T2

        if (k_msgq_put(&raw_queue, &msg, K_NO_WAIT) != 0)
        {
            ++queue_drops;                           // thread yetişemedi, mesaj kayıp
        }

        msg_len = 0;
    }
}

/* Hat hatalarını sayar, gelen baytları mesaja ekler. */
void uart_isr(const struct device *dev, void *user_data)
{
    (void)user_data;

    uart_irq_update(dev);                            // diğer uart_irq çağrılarından önce şart

    if (uart_err_check(dev) > 0)
    {
        ++rx_errors;                                 // taşma, çerçeve ya da gürültü hatası
    }

    if (uart_irq_rx_ready(dev) > 0)
    {
        uint8_t byte;

        while (uart_fifo_read(dev, &byte, 1) == 1)
        {
            add_byte(byte);
        }
    }
}

/* Linux'un haberini link thread'ine iletir. */
void ipcc_isr(const struct device *dev, void *user_data, uint32_t channel, volatile void *data)
{
    (void)dev;
    (void)user_data;
    (void)channel;
    (void)data;
    k_sem_give(&ipcc_sem);
}
