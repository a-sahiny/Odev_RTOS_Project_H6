/* A35 <-> M33 satır protokolü: rpmsg ile gelen baytları satıra, satırı LinkMsg'ye çevirir.
   Yalnız RPMsg'nin sahibi olan task çağırır; global durum yok, yeniden girilebilir. */
#include "link_proto.h"

#include <string.h>

typedef struct
{
    const char *name;                       // satırın ilk kelimesi
    LinkMsgType type;
    uint8_t count;                          // beklenen sayı alanı
    int32_t min[7];                         // alan başına alt sınır (dahil)
    int32_t max[7];                         // alan başına üst sınır (dahil)
} MsgRule;

/* Her mesajın alan sayısı ve aralığı; sınır dışı satır tümüyle reddedilir. */
static const MsgRule rules[] =
{
    { "HB",  LINK_MSG_HB,  1U, { 0 }, { 65535 } },
    { "TM",  LINK_MSG_TM,  7U, { 0, -32768, -32768, -32768, -32768, -32768, -32768 },
                               { 65535, 32767, 32767, 32767, 32767, 32767, 32767 } },
    { "AT",  LINK_MSG_AT,  4U, { -36000, -36000, -36000, 0 }, { 36000, 36000, 36000, 255 } },
    { "ST",  LINK_MSG_ST,  2U, { 0, 0 }, { 255, 255 } },
    { "LIM", LINK_MSG_LIM, 1U, { 100 }, { 18000 } },
};

/* Ondalık işaretli sayı okur, *end sayının bittiği yer; başarı 1. Taşmayı rakam rakam denetler. */
static int parse_i32(const char *text, const char **end, int32_t *value)
{
    int negative = 0;
    if (*text == '-')
    {
        negative = 1;
        ++text;
    }

    if ((*text < '0') || (*text > '9'))
    {
        return 0;                                    // en az bir rakam şart
    }

    int64_t acc = 0;
    while ((*text >= '0') && (*text <= '9'))
    {
        acc = (acc * 10) + (int64_t)(*text - '0');
        if (acc > 2147483648LL)
        {
            return 0;                                // int32 sınırını aştı
        }
        ++text;
    }

    acc = negative ? -acc : acc;
    if (acc > 2147483647LL)
    {
        return 0;
    }

    *value = (int32_t)acc;
    *end = text;
    return 1;
}

void line_rx_init(LineRx *rx)
{
    rx->len = 0U;
    rx->overflow = 0U;
}

int line_rx_push(LineRx *rx, char c, const char **line)
{
    if (c == '\r')
    {
        return 0;                                    // Linux tarafı \r\n gönderse de çalışsın
    }

    if (c != '\n')
    {
        if (rx->overflow != 0U)
        {
            return 0;                                // taşmış satırın kalanı atılıyor
        }
        if (rx->len >= (LINK_LINE_MAX - 1U))
        {
            rx->overflow = 1U;                       // '\0' için bir yer kalmalı
            return 0;
        }
        rx->buf[rx->len] = c;
        ++rx->len;
        return 0;
    }

    /* '\n': satır bitti; taşmış ya da boş satır mesaj değildir. */
    int ready = (rx->overflow == 0U) && (rx->len > 0U);
    rx->buf[rx->len] = '\0';
    rx->len = 0U;
    rx->overflow = 0U;
    if (ready)
    {
        *line = rx->buf;
    }
    return ready;
}

int link_parse(const char *line, LinkMsg *msg)
{
    const char *space = strchr(line, ' ');
    if (space == NULL)
    {
        return 0;                                    // her mesajın en az bir alanı var
    }

    size_t name_len = (size_t)(space - line);
    const MsgRule *rule = NULL;
    for (size_t i = 0U; i < (sizeof(rules) / sizeof(rules[0])); ++i)
    {
        if ((strlen(rules[i].name) == name_len) && (strncmp(rules[i].name, line, name_len) == 0))
        {
            rule = &rules[i];
            break;
        }
    }
    if (rule == NULL)
    {
        return 0;                                    // bilinmeyen mesaj
    }

    /* Önce bütün alanlar geçici yere okunur; satır geçerliyse *msg'ye yazılır. */
    LinkMsg out;
    memset(&out, 0, sizeof(out));
    out.type = rule->type;

    const char *p = space;
    for (uint8_t i = 0U; i < rule->count; ++i)
    {
        if (*p != ' ')
        {
            return 0;                                // alanlar tek boşlukla ayrılır
        }
        ++p;

        int32_t value = 0;
        if (parse_i32(p, &p, &value) == 0)
        {
            return 0;
        }
        if ((value < rule->min[i]) || (value > rule->max[i]))
        {
            return 0;                                // aralık dışı
        }
        out.v[i] = value;
    }

    if (*p != '\0')
    {
        return 0;                                    // fazla alan ya da çöp
    }

    *msg = out;
    return 1;
}
