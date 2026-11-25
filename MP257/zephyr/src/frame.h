#pragma once
#include <stddef.h>
#include <stdint.h>

#define FRAME_SOF      0xA5U                         // F746'daki raw_link.c ile aynı
#define FRAME_TELEM    0x54U                         // 'T': 23 bayt ham telemetri
#define FRAME_LINE     0x44U                         // 'D': sıra baytı + satırın bir parçası
#define FRAME_DATA_MAX 32U
#define PIECE_FIRST    0x80U                         // sıra baytında: satırın ilk parçası
#define LINE_MAX       544U                          // F746'nın en uzun satırı 529

typedef enum
{
    FRAME_WAIT_SOF,
    FRAME_WAIT_TYPE,
    FRAME_WAIT_LEN,
    FRAME_WAIT_DATA,
    FRAME_WAIT_CRC
} FrameState;

/* Bayt bayt toplanan çerçeve. */
typedef struct
{
    FrameState state;
    uint8_t raw[2U + FRAME_DATA_MAX];                // tür, uzunluk, veri
    size_t pos;                                      // alınan veri baytı
} FrameRx;

/* Parçalardan birleşen satır. */
typedef struct
{
    uint8_t buf[LINE_MAX];
    size_t len;
    uint8_t next;                                    // beklenen sıra
    int open;                                        // ilk parçası alınmış satır var
    uint32_t lost;                                   // eksik parça yüzünden atılan satır
} LineRx;

int frame_add(FrameRx *rx, uint8_t byte);                              // 1 tam çerçeve, -1 bozuk, 0 sürüyor
size_t line_add(LineRx *rx, const uint8_t *piece, size_t size);        // satır bittiyse uzunluğu, yoksa 0
