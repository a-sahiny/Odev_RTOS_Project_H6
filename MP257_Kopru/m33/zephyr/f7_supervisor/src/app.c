/* M33 Zephyr uygulaması: sup_thread (10 ms periyotlu gözetim, LED, buton).
   main_remote.c'deki TTY thread'i app_on_rx ile satırları verir; SUP/BTN satırları rpmsg_send ile A35'e gider. */
#include "app.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "link_proto.h"
#include "supervisor.h"

#include <string.h>

LOG_MODULE_REGISTER(f7sup, LOG_LEVEL_INF);

typedef struct
{
    uint8_t bad;                            // 1: satır ayrıştırılamadı
    uint32_t rx_us;                         // satırın TTY thread'ine geldiği an
    LinkMsg msg;
} RxItem;

/* 16 satır: 50 Hz TM + AT + HB, sup_thread 10 ms'de boşaltır; 4 kat pay. */
K_MSGQ_DEFINE(rx_q, sizeof(RxItem), 16, 4);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);   // LD9 turuncu, PH6
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);    // USER2, PC11, aktif düşük

static struct rpmsg_endpoint *tty_ept;      // TTY thread yazar (bir kez), sup_thread okur
static LineRx line_rx;                      // yalnız TTY thread kullanır
static uint32_t rx_drops;                   // rx_q dolu: kaybolan satır
static uint32_t tx_errors;                  // rpmsg_send hata döndü

/* Açılıştan beri mikrosaniye; çözünürlük sistem tick'i (prj.conf: 10 kHz = 100 us). */
static uint32_t time_us(void)
{
    return (uint32_t)k_ticks_to_us_floor64((uint64_t)k_uptime_ticks());
}

/* Satırı A35'e yollar; kanal henüz yoksa ya da gönderim başarısızsa sayar. */
static void send_line(const char *text, size_t len)
{
    struct rpmsg_endpoint *ept = tty_ept;
    if ((ept == NULL) || (len == 0U))
    {
        return;
    }
    /* rpmsg_send kendi içinde rpmsg aygıt kilidini alır; TTY thread'iyle aynı anda çağrılabilir. */
    if (rpmsg_send(ept, text, (int)len) < 0)
    {
        ++tx_errors;
    }
}

void app_set_tty_endpoint(struct rpmsg_endpoint *ept)
{
    line_rx_init(&line_rx);
    tty_ept = ept;
}

void app_on_rx(const void *data, size_t len)
{
    const char *bytes = data;
    uint32_t now = time_us();
    for (size_t i = 0U; i < len; ++i)
    {
        const char *line = NULL;
        if (line_rx_push(&line_rx, bytes[i], &line) == 0)
        {
            continue;
        }

        RxItem item;
        item.rx_us = now;
        item.bad = (link_parse(line, &item.msg) == 0) ? 1U : 0U;
        if (k_msgq_put(&rx_q, &item, K_NO_WAIT) != 0)
        {
            ++rx_drops;                              // beklemek RX tamponunu tutar; kaybı say
        }
    }
}

/* sup_thread: rx_q -> supervisor; 10 ms'de bir mod, LED, buton ve 500 ms'de bir SUP satırı. */
static void sup_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    if (!gpio_is_ready_dt(&led) || !gpio_is_ready_dt(&btn))
    {
        LOG_ERR("LED ya da buton hazir degil");
        return;
    }
    if ((gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE) != 0) || (gpio_pin_configure_dt(&btn, GPIO_INPUT) != 0))
    {
        LOG_ERR("GPIO ayari basarisiz");
        return;
    }

    Supervisor sup;
    sup_init(&sup, time_us());
    int rdy_sent = 0;

    int64_t next_ms = k_uptime_get();
    for (;;)
    {
        RxItem item;
        while (k_msgq_get(&rx_q, &item, K_NO_WAIT) == 0)
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

        if ((rdy_sent == 0) && (tty_ept != NULL))
        {
            send_line("RDY zephyr 1\n", 13U);       // kanal açılınca bir kez
            rdy_sent = 1;
        }

        uint32_t now = time_us();
        SupMode mode = sup_step(&sup, now);
        (void)gpio_pin_set_dt(&led, sup_led_on(mode, now));

        /* gpio_pin_get_dt aktif düşüklüğü DTS'den bilir: basılı = 1. */
        if (sup_button_sample(&sup, gpio_pin_get_dt(&btn) == 1) != 0)
        {
            send_line("BTN\n", 4U);
        }

        if (sup_report_due(&sup, now) != 0)
        {
            char line[LINK_LINE_MAX];
            size_t len = sup_format_report(&sup, now, line, sizeof(line));
            send_line(line, len);
        }

        next_ms += 10;                               // mutlak zamana uyku: periyot kaymaz
        k_sleep(K_TIMEOUT_ABS_MS(next_ms));
    }
}

/* 2048 bayt yığın (snprintf), öncelik 6 preemptive: OpenAMP thread'leri (coop 7-8) önce çalışır. */
K_THREAD_DEFINE(sup_tid, 2048, sup_thread, NULL, NULL, NULL, 6, 0, 0);
