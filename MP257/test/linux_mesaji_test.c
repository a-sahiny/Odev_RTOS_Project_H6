/* USART6 hattının PC'de birim testi: gerçek raw_link.c ve crc.c (F746), f746_tx.c ve frame.c (M33), donanım taklit.
 * Derleme ve çalıştırma: MP257\test\kos.ps1 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stub.h"

#include "../../App/Src/raw_link.c"
#undef LINUX_MSG_TYPE
#undef FRAME_SOF
#undef FRAME_TELEM
#undef FRAME_LINE
#undef PIECE_FIRST
#define rx_errors m33_rx_errors
#include "../zephyr/src/f746_tx.c"
#include "../zephyr/src/frame.c"

#define US_PER_BYTE (10.0 * 1e6 / 460800.0)
#define PIECES      ((529 + PIECE_CHARS - 1) / PIECE_CHARS) // en uzun satırın parçası

uint32_t tick_ms;
int irq_on;
int fifo_room;
uint8_t wire[16384];
int wire_len;
uint8_t *rx_target;
UART_HandleTypeDef huart6;
int notified;
static int fails;

// F746'nın hatta verdiği her çerçeve M33 ayrıştırıcısına gider
static FrameRx frx;
static LineRx lrx;
static uint8_t telem_out[32][APP_LEN_TELEM];
static int telem_n;
static char line_out[LINE_MAX];
static size_t line_out_len;
static int line_n;
static int bad_n;
static int corrupt_at = -1;                     // bu sıradaki çerçevenin bir baytı bozulur
static int drop_at = -1;                        // bu sıradaki çerçeve hiç ulaşmaz
static int inject_at = -1;                      // bu sıradaki çerçeve giderken telemetri gelir
static int frame_n;                             // hatta verilen çerçeve
static int telem_frame;                         // telemetrinin gittiği çerçeve sırası
static int dma_stuck;

static void check(const char *name, int ok)
{
    printf("%-6s %s\n", ok ? "GECTI" : "KALDI", name);
    fails += !ok;
}

/* M33 kesmesi gibi baytları çözer. */
static void m33_receive(const uint8_t *data, size_t size)
{
    for (size_t i = 0; i < size; ++i)
    {
        int r = frame_add(&frx, data[i]);

        if (r < 0)
        {
            ++bad_n;
        }
        else if (r > 0 && frx.raw[0] == FRAME_TELEM)
        {
            memcpy(telem_out[telem_n % 32], &frx.raw[2], APP_LEN_TELEM);
            ++telem_n;
            telem_frame = frame_n;
        }
        else if (r > 0)
        {
            size_t n = line_add(&lrx, &frx.raw[2], frx.raw[1]);

            if (n > 0)
            {
                memcpy(line_out, lrx.buf, n);
                line_out_len = n;
                ++line_n;
            }
        }
    }
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, const uint8_t *data, uint16_t size)
{
    if (u->gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    uint8_t copy[64];

    memcpy(copy, data, size);
    u->gState = HAL_UART_STATE_BUSY_TX;
    ++frame_n;

    if (frame_n == corrupt_at)
    {
        copy[size / 2] ^= 0x10;
    }

    if (frame_n != drop_at)
    {
        m33_receive(copy, size);
    }

    return HAL_OK;
}

/* Hattaki çerçevenin son biti çıktı. */
static void dma_done(void)
{
    huart6.gState = HAL_UART_STATE_READY;
    raw_link_tx_done_isr();
}

/* Task bekler: hat çalışır, bildirim gelince ya da hat takılınca döner. */
uint32_t ulTaskNotifyTake(int clear, uint32_t wait)
{
    static const uint8_t telem[APP_LEN_TELEM] = { 9 };

    (void)clear;

    if (wait == 0)
    {
        notified = 0;
        return 0;
    }

    while (!notified && !dma_stuck && huart6.gState == HAL_UART_STATE_BUSY_TX)
    {
        if (frame_n == inject_at)
        {
            inject_at = -1;
            raw_link_send(telem, 77, 0);
        }

        dma_done();
    }

    uint32_t result = (uint32_t)notified;

    notified = 0;

    return result;
}

static void drain(void)
{
    while (huart6.gState == HAL_UART_STATE_BUSY_TX)
    {
        dma_done();
    }
}

static int send_line(const char *text, size_t size)
{
    line_out_len = 0;
    return raw_link_send_line((const uint8_t *)text, size, 26);
}

// F746'nın komut akışına giren baytlar
static uint8_t fed[16384];
static size_t fed_len;
static int fed_lost;

void UartLink_FeedFromISR(const uint8_t *data, size_t size, BaseType_t lost_before, BaseType_t *woken)
{
    (void)woken;
    memcpy(&fed[fed_len], data, size);
    fed_len += size;
    fed_lost += lost_before;
}

static uint32_t dma_pos;

/* Baytları dairesel DMA gibi yazar: yarıda, sonda ve 'idle' baytta bir boşlukta kesme gelir. */
static void receive_idle(const uint8_t *data, size_t size, size_t idle)
{
    for (size_t i = 0; i < size; ++i)
    {
        rx_target[dma_pos++] = data[i];

        if (dma_pos == RX_DMA_BYTES / 2)
        {
            raw_link_rx_isr((uint16_t)dma_pos);
        }

        if (dma_pos == RX_DMA_BYTES)
        {
            raw_link_rx_isr(RX_DMA_BYTES);
            dma_pos = 0;
        }

        if ((i + 1) % idle == 0 || i + 1 == size)
        {
            raw_link_rx_isr((uint16_t)(dma_pos == 0 ? RX_DMA_BYTES : dma_pos));
        }
    }
}

static void receive(const uint8_t *data, int size)
{
    receive_idle(data, (size_t)size, 1);
}

/* M33'ün sırasındaki her şeyi gerçek kodla hatta verir; FIFO'da her kesmede 'room' bayt yer var. */
static void m33_run(int room)
{
    wire_len = 0;
    fifo_room = room;

    while (irq_on)
    {
        f746_tx_isr(uart);
    }
}

static void m33_send(int on, int room)
{
    f746_tx_state(on);
    m33_run(room);
    receive(wire, wire_len);
}

static void test_commands(void)
{
    static uint8_t data[2000];
    int same = 1;

    for (size_t i = 0; i < sizeof(data); ++i)
    {
        data[i] = (uint8_t)(i * 31 + i / 256 + (i % 5 == 0 ? 0xA5 - i * 31 - i / 256 : 0)); // her 5. bayt 0xA5
    }

    for (size_t size = 1; size <= sizeof(data); size += size < 100 ? 1 : 37)
    {
        fed_len = 0;
        f746_tx_command(data, size);

        if (size % 3 == 0)
        {
            f746_tx_state(1);                    // durum mesajı araya girer
        }

        m33_run(1 + (int)(size % 64));
        receive_idle(wire, (size_t)wire_len, 1 + size % 300);
        same &= fed_len == size && memcmp(fed, data, size) == 0;
    }

    check("E1 1..2000 bayt ayni, durum mesajlari arada", same && fed_lost == 0 && rx_lost == 0 && raw_link_linux_on());

    fed_len = 0;
    f746_tx_command(data, 93);                   // 3 çerçeve
    m33_run(64);
    wire[36 + 10] ^= 0x04;                       // ikincisi bozuk
    uint32_t bad = rx_bad;

    receive(wire, wire_len);
    check("E1 bozuk cerceve atilir, sayilir, sonraki sira farkini bildirir",
          fed_len == 62 && rx_bad > bad && rx_lost == 1 && fed_lost == 1);
    fed_len = 0;
    f746_tx_command(data, 40);
    m33_run(64);
    receive(wire, wire_len);
    check("E1 sonraki komut etkilenmez", fed_len == 40 && memcmp(fed, data, 40) == 0 && rx_lost == 1);

    for (int i = 0; i < 4; ++i)
    {
        f746_tx_command(data, 2000);             // 8000 bayt sığar
    }

    uint32_t drops = f746_tx_drops;

    f746_tx_command(data, 500);                  // sığmaz
    fed_len = 0;
    m33_run(64);
    receive_idle(wire, (size_t)wire_len, 1000);
    check("E1 dolu sirada mesaj butun atilir, F746 eksigi gorur", f746_tx_drops == drops + 1 && fed_len == 8000 && rx_lost == 2);

    // E2: bütün bölünmeler; DMA her konumdan başlar
    same = 1;

    for (size_t idle = 1; idle <= 2 * RX_DMA_BYTES; ++idle)
    {
        fed_len = 0;
        f746_tx_command(data, 310);
        m33_run(64);
        receive_idle(wire, (size_t)wire_len, idle);
        same &= fed_len == 310 && memcmp(fed, data, 310) == 0;
    }

    check("E2 yarim, tam ve bosluk olaylarinin her bolunmesinde bayt kaybi ve tekrari 0", same && rx_lost == 2);
}

static void test_linux_message(void)
{
    static const uint8_t on_msg[] = { 0xA5, 0x4C, 0x01, 0xFE };
    static const uint8_t off_msg[] = { 0xA5, 0x4C, 0x00, 0xFF };
    static const uint8_t bad_msg[] = { 0xA5, 0x4C, 0x01, 0xFF };
    static const uint8_t noise[] = { 0x00, 0xA5, 0x13, 0xA5, 0xA5, 0xFF, 0x4C };
    static const uint8_t cut[] = { 0xA5, 0x4C, 0x01 };

    raw_link_init();
    huart6.gState = HAL_UART_STATE_READY;
    check("C1 acilista kapali", !raw_link_linux_on());
    receive(on_msg, 4);
    check("C1 gecerli 'var'", raw_link_linux_on());
    receive(off_msg, 4);
    check("C2 'yok' hemen kapatir", !raw_link_linux_on());
    receive(bad_msg, 4);
    check("C1 bozuk kontrol bayti ret, sayildi", !raw_link_linux_on() && rx_bad == 1);
    receive(noise, sizeof(noise));
    receive(on_msg, 4);
    check("C1 gurultuden sonraki mesaj taninir", raw_link_linux_on());
    receive(off_msg, 4);
    receive(cut, 3);                            // yarım mesaj, ardından yeni mesajın başı
    receive(on_msg, 4);
    receive(on_msg, 4);
    check("C1 yarim mesajdan sonra en gec ikincisi taninir", raw_link_linux_on());

    tick_ms = 5000;
    receive(on_msg, 4);
    tick_ms = 5000 + 2500;
    check("C2 2500 ms'de acik", raw_link_linux_on());
    tick_ms = 5000 + 2501;
    check("C2 2501 ms'de kapali", !raw_link_linux_on());
    tick_ms = 5000;                             // sayaç turu: eski mesajın anı yeniden gelse de kapalı kalır
    check("C2 suresi dolan mesaj geri gelmez", !raw_link_linux_on());
    tick_ms = 0xFFFFFF00U;
    receive(on_msg, 4);
    tick_ms = 0x100U;
    check("C2 sayac tasmasinda acik (512 ms)", raw_link_linux_on());

    huart6.RxState = HAL_UART_STATE_READY;      // taşma HAL'de almayı durdurdu
    rx_target = NULL;
    raw_link_error_isr();
    dma_pos = 0;
    check("C1 hata sonrasi alma yeniden kurulur", rx_target != NULL && rx_pos == 0);

    tick_ms = 10000;
    m33_send(0, 64);
    check("C3 M33 'yok' -> F746 kapali", !raw_link_linux_on() && wire_len == 4);
    m33_send(1, 64);
    check("C3 M33 'var' -> F746 acik", raw_link_linux_on() && wire_len == 4);
    m33_send(0, 1);
    check("C3 FIFO bayt bayt bosalirken de ayni", !raw_link_linux_on() && wire_len == 4 && !irq_on);
    f746_tx_state(1);
    f746_tx_state(0);
    m33_run(64);
    receive(wire, wire_len);
    check("C3 ust uste iki istekte sonuncusu gider", !raw_link_linux_on() && wire_len == 4);
    huart6.RxState = 0x22U;                     // alma sürüyor
}

static void test_lines(void)
{
    static char text[LINE_MAX];
    int same = 1;

    for (size_t size = 1; size <= 529; ++size)
    {
        for (size_t i = 0; i + 1 < size; ++i)
        {
            text[i] = (char)('!' + (i * 7 + size) % 90);
        }

        text[size - 1] = '\n';
        same &= send_line(text, size) == 1 && line_out_len == size && memcmp(line_out, text, size) == 0;
        drain();
    }

    check("D1 1..529 karakterlik satirlar ayni cikar", same && line_n == 529 && bad_n == 0 && lrx.lost == 0);

    int before = line_n;

    corrupt_at = frame_n + 5;
    check("D1 F746 bozulmayi bilmez, satir yollanmis sayilir", send_line(text, 529) == 1);
    drain();
    check("D1 bozuk CRC: satir atilir, sayilir", line_n == before && lrx.lost == 1 && bad_n >= 1);
    check("D1 sonraki satir etkilenmez", send_line(text, 529) == 1 && line_out_len == 529 && line_n == before + 1);
    drop_at = frame_n + (int)PIECES;
    send_line(text, 529);
    drop_at = -1;
    check("D1 son parca kayip: satir cikmaz", line_n == before + 1);
    send_line(text, 529);
    check("D1 kaybin ardindan gelen satir cikar, kayip sayilir", line_n == before + 2 && lrx.lost == 2);

    static const uint8_t noise[] = { 0xA5, 0xA5, 0x54, 0x00, 0x44, 0xA5, 0x44, 0x05, 0x01 };

    m33_receive(noise, sizeof(noise));
    frx.state = FRAME_WAIT_SOF;                 // 2 ms sessizlik: kesme yarım çerçeveyi atar
    text[199] = '\n';
    check("D1 gurultuden sonra satir cikar", send_line(text, 200) == 1 && line_out_len == 200 && line_n == before + 3);
}

static void test_priority(void)
{
    static const uint8_t telem[APP_LEN_TELEM] = { 1, 2, 3, 0xA5, 0x44 };
    static char text[529];
    double worst = 0;

    memset(text, 'x', sizeof(text));
    text[528] = '\n';
    telem_n = 0;
    raw_link_send(telem, 5, 0);
    drain();
    check("D2 bos hatta telemetri hemen ve ayni", telem_n == 1 && memcmp(telem_out[0], telem, APP_LEN_TELEM) == 0);

    int ok = 1;

    for (int at = 1; at <= (int)PIECES; ++at)            // satırın her parçasında telemetri gelsin
    {
        int base = frame_n;
        int seen = telem_n;

        inject_at = base + at;
        ok &= send_line(text, 529) == 1 && line_out_len == 529;
        drain();
        ok &= telem_n == seen + 1 && telem_frame == base + at + 1;
        double wait_us = (FRAME_OVERHEAD + 1 + PIECE_CHARS) * US_PER_BYTE; // en kötü: parça yeni başlamışken

        worst = wait_us > worst ? wait_us : worst;
    }

    check("D2 telemetri hep bir sonraki cerceve, satir butun", ok);
    printf("       en uzun bekleme %.0f us (bir parca), satir 529 karakter %.1f ms\n", worst,
           (529 + PIECES * 5) * US_PER_BYTE / 1000.0);
    check("D2 en uzun bekleme <= 800 us", worst <= 800.0);

    uint32_t drops = tx_drops;

    raw_link_send(telem, 6, 0);                 // hatta
    raw_link_send(telem, 7, 0);                 // sırada
    raw_link_send(telem, 8, 0);                 // düşer
    check("D2 uc telemetri ust uste: biri duser, sayilir", tx_drops == drops + 1);
    drain();
    check("D2 last_sent son gidenin SEQ'i", (raw_link_last() >> 16) == 7);

    dma_stuck = 1;                              // DMA takıldı
    check("D2 takilan hatta satir suresi dolar", send_line(text, 529) == 0);
    dma_stuck = 0;
    drain();
    int seen = line_n;
    check("D2 takilmadan sonra satir yine gider", send_line(text, 529) == 1 && line_n == seen + 1);

    tx_kind = TX_TELEM;                         // telemetri giderken DMA hatası
    huart6.gState = HAL_UART_STATE_READY;
    raw_link_error_isr();
    check("D2 DMA hatasindan sonra hat kilitlenmez", tx_kind == TX_IDLE);
}

int main(void)
{
    test_linux_message();
    test_lines();
    test_priority();
    test_commands();
    printf("SONUC: %s\n", fails ? "KALDI" : "GECTI");

    return fails != 0;
}
