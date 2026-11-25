/* Ham mesajdan Host'un beklediği TELEM paketini kurar; donanım ve kesme kullanmaz. */
#include "packet.h"
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <string.h>

#define PKT_SOF  0xA5U
#define PKT_VER  0x01U
#define PKT_TELEM 0x30U                               // Host'taki TELEM türü
#define TELEM_CRC8_POLY 0x07U                         // CRC-8/SMBUS: yansıtma ve xorout yok, başlangıç 0

/* Paketin SEQ'ini döndürür: F746'nın prev_seq'i + 1; önceki mesaj bilgisi yoksa 0. */
uint16_t packet_seq(const uint8_t *raw)
{
    if (!(raw[RAW_FLAGS_OFFSET] & RAW_FLAG_PREV))
    {
        return 0;
    }

    return (uint16_t)(sys_get_be16(&raw[RAW_PREV_SEQ_OFFSET]) + 1U);
}

/* 23 baytlık ham mesajı ve önceki paketin M33 süresini 35 baytlık pakete çevirir. */
void packet_build(uint8_t *out, const uint8_t *raw, uint16_t m33_us)
{
    out[0] = PKT_SOF;
    out[1] = PKT_VER;
    out[2] = PKT_TELEM;
    out[3] = 0;                                       // bayrak yok: veri paketi
    sys_put_be16(packet_seq(raw), &out[4]);
    out[6] = 0;                                       // REQ_ID: veride 0
    out[7] = 0;
    out[8] = MSG_SIZE + M33_US_SIZE;
    memcpy(&out[PKT_HEADER_SIZE], raw, MSG_SIZE);
    sys_put_be16(m33_us, &out[PKT_HEADER_SIZE + MSG_SIZE]);  // bir paket kendi süresini taşıyamaz: öncekinin süresi
    out[PKT_SIZE - 1U] = crc8(out, PKT_SIZE - 1U, TELEM_CRC8_POLY, 0x00, false);
}
