"""CRC'ler: kartın crc.c dosyasıyla aynı tanımlar (tasarım D6, 13.4)."""

import struct
import zlib


def _make_crc8_table():
    table = []
    for byte in range(256):
        crc = byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return bytes(table)


_CRC8_TABLE = _make_crc8_table()


def crc8(data: bytes) -> int:
    """CRC-8/SMBUS: poly 0x07, init 0, yansıtma yok, xorout 0. "123456789" -> 0xF4."""
    crc = 0
    for b in data:
        crc = _CRC8_TABLE[crc ^ b]
    return crc


def crc32_eth(data: bytes) -> int:
    """IEEE 802.3 CRC-32 (zlib ile aynı). "123456789" -> 0xCBF43926."""
    return zlib.crc32(data) & 0xFFFFFFFF


def fcs_bytes(frame: bytes) -> bytes:
    """Hatta giden FCS: CRC-32 en düşük bayt önce."""
    return struct.pack("<I", crc32_eth(frame))


def fcs_ok(frame_with_fcs: bytes) -> bool:
    if len(frame_with_fcs) < 5:
        return False
    return fcs_bytes(frame_with_fcs[:-4]) == frame_with_fcs[-4:]
