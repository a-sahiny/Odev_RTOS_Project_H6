/* A35 <-> M33 satır protokolü: bayt -> satır birleştirme, satır -> mesaj, mesaj -> satır.
   RTOS'suz saf C; FreeRTOS ve Zephyr varyantı aynı dosyayı kullanır. Tanım: protocol/A35_M33_Protokolu.md */
#pragma once
#include <stdint.h>
#include <stddef.h>

#define LINK_LINE_MAX 96U                   // en uzun satır + '\0'; TM satırı ~60 karakter

typedef enum
{
    LINK_MSG_HB = 0,                        // A35 kalp atışı: HB <seq>
    LINK_MSG_TM,                            // telemetri: TM <seq> <ax> <ay> <az> <gx> <gy> <gz>
    LINK_MSG_AT,                            // açı: AT <roll> <pitch> <yaw> <flags> (0,01 derece)
    LINK_MSG_ST,                            // F746 durumu: ST <state> <why>
    LINK_MSG_LIM                            // eğim alarm sınırı: LIM <cdeg>
} LinkMsgType;

typedef struct
{
    LinkMsgType type;
    int32_t v[7];                           // alanlar satırdaki sırayla; kullanılmayanlar 0
} LinkMsg;

typedef struct
{
    char buf[LINK_LINE_MAX];
    size_t len;
    uint8_t overflow;                       // 1: satır taştı, sonraki '\n'e kadar atılıyor
} LineRx;

void line_rx_init(LineRx *rx);                                  // birleştiriciyi boşaltır
int line_rx_push(LineRx *rx, char c, const char **line);       // satır tamamlanınca 1 ve *line
int link_parse(const char *line, LinkMsg *msg);                 // geçerli satırda 1; hatada *msg değişmez
