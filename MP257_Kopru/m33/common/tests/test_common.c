/* PC'de (gcc) çalışan birim testleri: link_proto ve supervisor; kart gerekmez.
   Derleme: gcc -std=c11 -Wall -Wextra -I.. ../link_proto.c ../supervisor.c test_common.c -o test_common */
#include "link_proto.h"
#include "supervisor.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

#define CHECK(cond) do { if (!(cond)) { printf("HATA %s:%d: %s\n", __FILE__, __LINE__, #cond); ++failures; } } while (0)

/* Metni birleştiriciye verir, tamamlanan son satırı ayrıştırır; ayrıştırma sonucu döner. */
static int feed(LineRx *rx, const char *text, LinkMsg *msg)
{
    int result = -1;
    for (const char *p = text; *p != '\0'; ++p)
    {
        const char *line = NULL;
        if (line_rx_push(rx, *p, &line) != 0)
        {
            result = link_parse(line, msg);
        }
    }
    return result;
}

static void test_parse(void)
{
    LineRx rx;
    line_rx_init(&rx);
    LinkMsg m;

    CHECK(feed(&rx, "HB 7\n", &m) == 1);
    CHECK((m.type == LINK_MSG_HB) && (m.v[0] == 7));

    CHECK(feed(&rx, "TM 65535 -32768 1 2 3 4 32767\r\n", &m) == 1);
    CHECK((m.type == LINK_MSG_TM) && (m.v[1] == -32768) && (m.v[6] == 32767));

    CHECK(feed(&rx, "AT -1500 200 0 3\n", &m) == 1);
    CHECK((m.type == LINK_MSG_AT) && (m.v[0] == -1500) && (m.v[3] == 3));

    CHECK(feed(&rx, "TM 1 2 3\n", &m) == 0);               // eksik alan
    CHECK(feed(&rx, "TM 1 2 3 4 5 6 7 8\n", &m) == 0);     // fazla alan
    CHECK(feed(&rx, "HB 65536\n", &m) == 0);               // aralık dışı
    CHECK(feed(&rx, "HB  1\n", &m) == 0);                  // çift boşluk
    CHECK(feed(&rx, "XX 1\n", &m) == 0);                   // bilinmeyen
    CHECK(feed(&rx, "LIM 99999999999\n", &m) == 0);        // int32 taşması
    CHECK(feed(&rx, "HB\n", &m) == 0);                     // alansız

    /* Taşan satır atılır, sonraki satır sağlam gelir. */
    char longline[200];
    memset(longline, 'A', sizeof(longline) - 2U);
    longline[sizeof(longline) - 2U] = '\n';
    longline[sizeof(longline) - 1U] = '\0';
    CHECK(feed(&rx, longline, &m) == -1);
    CHECK(feed(&rx, "HB 9\n", &m) == 1);
    CHECK(m.v[0] == 9);
}

static void test_supervisor(void)
{
    Supervisor s;
    uint32_t t = 0xFFFF0000U;                              // taşmaya yakın başla: fark hesabı sınansın
    sup_init(&s, t);
    CHECK(sup_step(&s, t) == SUP_WAIT_A35);

    LinkMsg hb = { LINK_MSG_HB, { 1 } };
    sup_on_msg(&s, &hb, t);
    CHECK(sup_step(&s, t) == SUP_NO_DATA);

    LinkMsg tm = { LINK_MSG_TM, { 10, 0, 0, 0, 0, 0, 0 } };
    sup_on_msg(&s, &tm, t);
    t += 100000U;                                          // 100 ms sonra (taşma burada olur)
    tm.v[0] = 13;                                          // 11 ve 12 kayıp
    sup_on_msg(&s, &hb, t);
    sup_on_msg(&s, &tm, t);
    CHECK(sup_step(&s, t) == SUP_RUN);
    CHECK(s.tm_gaps == 2U);
    CHECK(s.dt_max_us == 100000U);

    LinkMsg at = { LINK_MSG_AT, { 3500, 0, 0, 1 } };       // 35 derece > 30
    sup_on_msg(&s, &at, t);
    CHECK(sup_step(&s, t) == SUP_ALARM);

    LinkMsg st = { LINK_MSG_ST, { 0x05, 0x01 } };          // NET_READY | FAULT, IMU
    sup_on_msg(&s, &st, t);
    CHECK(sup_step(&s, t) == SUP_F7_FAULT);

    t += 600000U;                                          // HB yok > 500 ms
    CHECK(sup_step(&s, t) == SUP_A35_LOST);

    char line[LINK_LINE_MAX];
    CHECK(sup_report_due(&s, t) == 1);
    CHECK(sup_format_report(&s, t, line, sizeof(line)) > 0U);
    CHECK(strncmp(line, "SUP 5 ", 6) == 0);
    CHECK(sup_report_due(&s, t) == 0);

    /* Buton: 3 kararlı örnek bir basış; basılı tutmak tekrar saymaz. */
    CHECK(sup_button_sample(&s, 1) == 0);
    CHECK(sup_button_sample(&s, 1) == 0);
    CHECK(sup_button_sample(&s, 1) == 1);
    CHECK(sup_button_sample(&s, 1) == 0);
    CHECK(sup_button_sample(&s, 0) == 0);
    CHECK(sup_button_sample(&s, 1) == 0);

    CHECK(sup_led_on(SUP_RUN, 123456U) == 1);
    CHECK(sup_led_on(SUP_ALARM, 60000U) == 0);
}

int main(void)
{
    test_parse();
    test_supervisor();
    printf("%s (%d hata)\n", (failures == 0) ? "TAMAM" : "BASARISIZ", failures);
    return (failures == 0) ? 0 : 1;
}
