/* Kesmeler: USART6 çerçevelerini main ve line thread'lerine, IPCC haberini link thread'ine verir; süre dolunca göndermeyi kapatır. */
#include "app.h"
#include "f746_tx.h"
#include "ipcc.h"
#include "rpmsg_link.h"
#include <zephyr/drivers/uart.h>
#include <string.h>

static FrameRx rx;                                   // toplanan çerçeve
static uint32_t first_cycles;                        // T1
static uint32_t last_byte_cycles;                    // son baytın alındığı an

/* Tam çerçeveyi türüne göre thread'ine verir; kuyruk doluysa kayıp sayılır. */
static void give_frame(uint32_t now)
{
    uint8_t type = rx.raw[0];
    uint8_t size = rx.raw[1];

    if (type == FRAME_TELEM && size == MSG_SIZE)
    {
        RawMsg msg = { .first_cycles = first_cycles, .last_cycles = now };

        memcpy(msg.data, &rx.raw[2], MSG_SIZE);

        if (k_msgq_put(&raw_queue, &msg, K_NO_WAIT) != 0)
        {
            ++queue_drops;                           // thread yetişemedi, mesaj kayıp
        }
    }
    else if (type == FRAME_LINE)
    {
        Piece piece = { .size = size };

        memcpy(piece.data, &rx.raw[2], size);

        if (k_msgq_put(&piece_queue, &piece, K_NO_WAIT) != 0)
        {
            ++piece_drops;                           // eksik parça line_add'de satırı düşürür
        }
    }
    else
    {
        ++bad_len;
    }
}

/* Bir baytı çerçeveye ekler; tam çerçeveyi iletir. */
static void add_byte(uint8_t byte)
{
    uint32_t now = k_cycle_get_32();

    if (rx.state != FRAME_WAIT_SOF && now - last_byte_cycles > k_us_to_cyc_floor32(MSG_GAP_US))
    {
        ++bad_len;                                   // yarım kalan çerçeve atılır
        rx.state = FRAME_WAIT_SOF;
    }

    last_byte_cycles = now;
    int result = frame_add(&rx, byte);

    if (rx.state == FRAME_WAIT_TYPE)
    {
        first_cycles = now;                          // T1: bu bayt SOF
    }

    if (result > 0)
    {
        give_frame(now);                             // T2
    }
    else if (result < 0)
    {
        ++bad_len;
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

    if (uart_irq_tx_ready(dev) > 0)
    {
        f746_tx_isr(dev);
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

/* Linux'un '1' komutu yenilenmedi: göndermeyi kapatır. */
void lease_isr(struct k_timer *timer)
{
    (void)timer;
    link_on = 0;
    ++link_expired;
}
