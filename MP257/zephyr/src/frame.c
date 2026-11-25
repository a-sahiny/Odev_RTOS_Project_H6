/* F746'nın USART6 çerçevelerini çözer ve satır parçalarını birleştirir; donanım ve kesme kullanmaz. */
#include "frame.h"
#include <zephyr/sys/crc.h>
#include <string.h>

#define FRAME_CRC8_POLY 0x07U                        // CRC-8/SMBUS, başlangıç 0

/* Bir baytı ekler; bozuk çerçevenin son baytı yeni çerçevenin başı olabilir. */
int frame_add(FrameRx *rx, uint8_t byte)
{
    switch (rx->state)
    {
    case FRAME_WAIT_SOF:
        rx->state = byte == FRAME_SOF ? FRAME_WAIT_TYPE : FRAME_WAIT_SOF;
        return 0;

    case FRAME_WAIT_TYPE:
        if (byte == FRAME_TELEM || byte == FRAME_LINE)
        {
            rx->raw[0] = byte;
            rx->state = FRAME_WAIT_LEN;
            return 0;
        }
        break;

    case FRAME_WAIT_LEN:
        if (byte >= 1U && byte <= FRAME_DATA_MAX)
        {
            rx->raw[1] = byte;
            rx->pos = 0;
            rx->state = FRAME_WAIT_DATA;
            return 0;
        }
        break;

    case FRAME_WAIT_DATA:
        rx->raw[2U + rx->pos] = byte;
        ++rx->pos;

        if (rx->pos == rx->raw[1])
        {
            rx->state = FRAME_WAIT_CRC;
        }
        return 0;

    case FRAME_WAIT_CRC:
        if (byte == crc8(rx->raw, 2U + rx->raw[1], FRAME_CRC8_POLY, 0x00, false))
        {
            rx->state = FRAME_WAIT_SOF;
            return 1;
        }
        break;
    }

    rx->state = byte == FRAME_SOF ? FRAME_WAIT_TYPE : FRAME_WAIT_SOF;

    return -1;
}

/* Parçayı satıra ekler; piece[0] sıra baytıdır. Sırası tutmayan parçada satır atılır, sonraki satır etkilenmez. */
size_t line_add(LineRx *rx, const uint8_t *piece, size_t size)
{
    uint8_t seq = piece[0] & (uint8_t)~PIECE_FIRST;
    int first = (piece[0] & PIECE_FIRST) != 0;
    int in_order = seq == rx->next;

    rx->next = (uint8_t)((seq + 1U) & 0x7FU);
    ++piece;
    --size;

    if (first)
    {
        if (rx->open)
        {
            ++rx->lost;                              // önceki satırın sonu gelmedi
        }

        rx->open = 1;
        rx->len = 0;
    }
    else if (!rx->open)
    {
        return 0;                                    // atılan satırın kalanı
    }
    else if (!in_order || rx->len + size > LINE_MAX)
    {
        rx->open = 0;
        ++rx->lost;
        return 0;
    }

    memcpy(&rx->buf[rx->len], piece, size);
    rx->len += size;

    if (size == 0 || rx->buf[rx->len - 1U] != '\n')
    {
        return 0;
    }

    rx->open = 0;

    return rx->len;
}
