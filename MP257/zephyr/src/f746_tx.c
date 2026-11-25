/* F746'ya USART6'dan "Linux var / yok" mesajını ve PC'nin Linux'tan gelen baytlarını yollar:
   link thread'i sıraya koyar, USART6 kesmesi hatta verir. */
#include "f746_tx.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/ring_buffer.h>

#define LINK_SOF       0xA5U                         // F746'daki raw_link.c ile aynı
#define LINUX_MSG_TYPE 0x4CU                         // 'L'
#define CMD_TYPE       0x43U                         // 'C'
#define LINK_CRC8_POLY 0x07U                         // CRC-8/SMBUS, başlangıç 0

volatile uint32_t f746_tx_count;                     // hatta verilen durum mesajı
volatile uint32_t f746_tx_cmds;                      // hatta verilen komut çerçevesi
volatile uint32_t f746_tx_drops;                     // sıra dolu olduğu için atılan komut mesajı

// Link thread'i yazar, USART6 kesmesi okur; 8192 bayt 0,2 s'lik hat
RING_BUF_DECLARE(cmd_ring, 8192);

static const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(usart6));
static volatile uint32_t state_wanted;               // yollanacak durum mesajı: 0 yok, 1 "yok", 2 "var"
static volatile uint32_t lost;                       // komut atıldı: sonraki çerçevenin sırası atlar
static uint8_t out[CMD_FRAME_MAX];                   // hattaki mesaj; yalnız kesme kullanır
static size_t out_len;
static size_t out_pos;
static uint8_t cmd_seq;                              // 7 bit

/* Mesajı kurar; son bayt durumun tersidir, bozuk mesaj F746'da tanınmaz. */
void linux_msg_build(uint8_t *msg, int on)
{
    msg[0] = LINK_SOF;
    msg[1] = LINUX_MSG_TYPE;
    msg[2] = on ? 1U : 0U;
    msg[3] = (uint8_t)~msg[2];
}

/* msg[4]'ten başlayan n baytı komut çerçevesine alır; çerçevenin uzunluğunu döndürür. */
size_t cmd_frame_build(uint8_t *msg, uint8_t seq, size_t n)
{
    msg[0] = LINK_SOF;
    msg[1] = CMD_TYPE;
    msg[2] = (uint8_t)(1U + n);
    msg[3] = seq;
    msg[4U + n] = crc8(&msg[1], 3U + n, LINK_CRC8_POLY, 0x00, false);

    return 5U + n;
}

/* Durum mesajını ister; öncekisi yollanmadıysa yenisi geçerlidir. */
void f746_tx_state(int on)
{
    state_wanted = on ? 2U : 1U;
    uart_irq_tx_enable(uart);
}

/* Baytları sıraya koyar; sığmazsa mesajın tamamı atılır ve sayılır. */
void f746_tx_command(const uint8_t *data, size_t size)
{
    if (ring_buf_space_get(&cmd_ring) < size)
    {
        ++f746_tx_drops;
        lost = 1;                                    // F746 eksik olduğunu sıradan anlar
        return;
    }

    (void)ring_buf_put(&cmd_ring, data, (uint32_t)size);
    uart_irq_tx_enable(uart);
}

/* Sıradaki mesajı out'a kurar; yollanacak yoksa 0. Durum mesajı önce gider. */
static size_t next_message(void)
{
    uint32_t wanted = state_wanted;

    if (wanted != 0)
    {
        state_wanted = 0;
        linux_msg_build(out, wanted == 2U);
        ++f746_tx_count;

        return LINUX_MSG_SIZE;
    }

    size_t n = ring_buf_get(&cmd_ring, &out[4], CMD_CHARS);

    if (n == 0)
    {
        return 0;
    }

    if (lost)
    {
        lost = 0;
        cmd_seq = (uint8_t)((cmd_seq + 1U) & 0x7FU);
    }

    size_t size = cmd_frame_build(out, cmd_seq, n);

    cmd_seq = (uint8_t)((cmd_seq + 1U) & 0x7FU);
    ++f746_tx_cmds;

    return size;
}

/* Sığan baytları FIFO'ya yazar; yollanacak kalmayınca yazma kesmesini kapatır. */
void f746_tx_isr(const struct device *dev)
{
    if (out_pos == out_len)
    {
        out_len = next_message();
        out_pos = 0;

        if (out_len == 0)
        {
            uart_irq_tx_disable(dev);
            return;
        }
    }

    out_pos += (size_t)uart_fifo_fill(dev, &out[out_pos], (int)(out_len - out_pos));
}
