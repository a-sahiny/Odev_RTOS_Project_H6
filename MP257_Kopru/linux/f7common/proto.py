"""F746 telemetri istasyonunun protokolü (Odev_RTOS_Project_H6 ile birebir aynı tanımlar).

Uygulama paketi (F746 App/Inc/pkt_format.h):
    SOF(0xA5) VER(0x01) TYPE FLAGS SEQ(2) REQ_ID(2) LEN | PAYLOAD | CRC8   (çok baytlı alanlar big-endian)
UART satırı (F746 line_rx.c):
    FRAME 1 <Ethernet çerçevesi + FCS, büyük harf hex>\n   ya da   {json}\n
Yalnız Python standart kütüphanesi; kartta pip gerekmez.
"""

import struct
import zlib

# ---------------------------------------------------------------- adresler (F746 app_config.h, Host ethframe.py)
BOARD_IP = "192.168.50.2"
HOST_IP = "192.168.50.1"            # MP257'deki TAP arayüzünün adresi
HOST_MAC = "02:00:00:00:00:01"      # eski PC host'unun kullandığı MAC; F746 için fark etmez
HOST_PORT = 6000                    # hub'ın UDP portu: telemetri buraya gelir
PORT_CTRL = 5000
PORT_LOW = 5001
PORT_TELEM = 5002

# ---------------------------------------------------------------- UART satır sınırları (F746 app_config.h)
UART_BAUD = 625000
LINE_MAX = 540
FRAME_MIN = 60                      # FCS hariç; kısa çerçeve dolgu ile 60'a tamamlanır
FRAME_MAX = 256                     # FCS hariç; TAP MTU'su 256 - 14 = 242
FCS_BYTES = 4

# ---------------------------------------------------------------- paket
SOF = 0xA5
VERSION = 0x01
OVERHEAD = 10
MAX_PAYLOAD = 200
FLAG_RESP = 0x01
FLAG_ERR = 0x02

CMD_START = 0x01
CMD_STOP = 0x02
CMD_SET_RATE = 0x03
CMD_GET_STATS = 0x04
CMD_GET_IMU = 0x05
ENV = 0x20
BULK = 0x21
CAL = 0x22
TELEM = 0x30

TYPE_NAMES = {CMD_START: "START", CMD_STOP: "STOP", CMD_SET_RATE: "SET_RATE", CMD_GET_STATS: "GET_STATS",
              CMD_GET_IMU: "GET_IMU", ENV: "ENV", BULK: "BULK", CAL: "CAL", TELEM: "TELEM"}
STATUS_NAMES = {0: "OK", 1: "E_LEN", 2: "E_RANGE", 3: "E_STATE", 4: "E_CONFLICT", 5: "E_TYPE", 6: "E_REQID"}

STATE_NET_READY = 0x01
STATE_MEAS_EN = 0x02
STATE_FAULT = 0x04
FAULT_NAMES = {0x01: "IMU", 0x02: "POOL", 0x04: "UART_ORE", 0x08: "UART_TX_STALL", 0x10: "HEAP"}

RATE_MIN_HZ = 1
RATE_MAX_HZ = 50
STATS_PAGES = 4
IMU_REPLY_LEN = 35


def _crc8_table():
    table = []
    for byte in range(256):
        crc = byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return bytes(table)


_CRC8 = _crc8_table()


def crc8(data: bytes) -> int:
    """CRC-8/SMBUS (poly 0x07, init 0). "123456789" -> 0xF4."""
    crc = 0
    for b in data:
        crc = _CRC8[crc ^ b]
    return crc


def fcs(frame: bytes) -> bytes:
    """Ethernet FCS: CRC-32 (zlib ile aynı), en düşük bayt önce."""
    return struct.pack("<I", zlib.crc32(frame) & 0xFFFFFFFF)


class PacketError(ValueError):
    pass


def encode(type_: int, seq: int, req_id: int, payload: bytes = b"", flags: int = 0) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise PacketError("payload > 200")
    head = struct.pack(">BBBBHHB", SOF, VERSION, type_, flags, seq & 0xFFFF, req_id & 0xFFFF, len(payload))
    body = head + payload
    return body + bytes([crc8(body)])


def decode(data: bytes):
    """F746 Pkt_Decode sırası: boy, SOF, VER, LEN, CRC, FLAGS. Döner: (type, flags, seq, req_id, payload)."""
    if len(data) < OVERHEAD:
        raise PacketError("short")
    if data[0] != SOF:
        raise PacketError("sof")
    if data[1] != VERSION:
        raise PacketError("version")
    ln = data[8]
    if ln > MAX_PAYLOAD or len(data) != OVERHEAD + ln:
        raise PacketError("length")
    if crc8(data[:-1]) != data[-1]:
        raise PacketError("crc")
    if data[3] & ~(FLAG_RESP | FLAG_ERR):
        raise PacketError("flags")
    seq, req_id = struct.unpack(">HH", data[4:8])
    return data[2], data[3], seq, req_id, bytes(data[9:9 + ln])


def parse_response(payload: bytes) -> dict:
    """Komut cevabının ortak 6 baytı: STATUS EPOCH(2) STATE LAT_US(2), sonra türe özgü kısım."""
    if len(payload) < 6:
        raise PacketError("resp_short")
    status = payload[0]
    epoch, = struct.unpack(">H", payload[1:3])
    lat_us, = struct.unpack(">H", payload[4:6])
    return {"status": status, "status_name": STATUS_NAMES.get(status, "0x%02X" % status),
            "epoch": epoch, "state": payload[3], "lat_us": lat_us, "extra": payload[6:]}


def parse_telem(payload: bytes) -> dict:
    """TELEM: idx u32, tick u32, ax ay az gx gy gz i16 (ham LSB)."""
    if len(payload) != 20:
        raise PacketError("telem_len")
    idx, tick, ax, ay, az, gx, gy, gz = struct.unpack(">IIhhhhhh", payload)
    return {"idx": idx, "tick": tick, "acc": [ax, ay, az], "gyro": [gx, gy, gz]}


def parse_imu_reply(extra: bytes) -> dict:
    """GET_IMU cevabı (ortak kısımdan sonra 35 B): açılar 0,01 derece."""
    if len(extra) != IMU_REPLY_LEN:
        raise PacketError("imu_len")
    idx, tick, ax, ay, az, gx, gy, gz, r, p, y, flags, prq, pus, t_rx = struct.unpack(">IIhhhhhhhhhBHHI", extra)
    return {"idx": idx, "tick": tick, "acc": [ax, ay, az], "gyro": [gx, gy, gz],
            "roll_cdeg": r, "pitch_cdeg": p, "yaw_cdeg": y, "flags": flags,
            "prev_req_id": prq, "prev_us": pus}


def parse_stats_page(extra: bytes) -> dict:
    if len(extra) < 6:
        raise PacketError("stats_short")
    page = extra[0]
    tick, = struct.unpack(">I", extra[1:5])
    n = extra[5]
    if len(extra) != 6 + 4 * n:
        raise PacketError("stats_len")
    return {"page": page, "tick": tick, "values": list(struct.unpack(">%dI" % n, extra[6:6 + 4 * n]))}


# ---------------------------------------------------------------- UART satırı <-> Ethernet çerçevesi
def frame_to_line(frame: bytes) -> bytes:
    """TAP'tan gelen çerçeveyi F746'nın kabul ettiği satıra çevirir (60 B'ye dolgu + FCS)."""
    if len(frame) > FRAME_MAX:
        raise PacketError("frame_long")
    if len(frame) < FRAME_MIN:
        frame = frame + bytes(FRAME_MIN - len(frame))
    return b"FRAME 1 " + (frame + fcs(frame)).hex().upper().encode() + b"\n"


def line_to_frame(line: bytes) -> bytes:
    """F746'dan gelen 'FRAME 1 <hex>' satırını doğrulayıp FCS'siz çerçeveyi döndürür."""
    if not line.startswith(b"FRAME 1 "):
        raise PacketError("not_frame")
    try:
        raw = bytes.fromhex(line[8:].decode("ascii"))
    except ValueError:
        raise PacketError("hex")
    if len(raw) < FRAME_MIN + FCS_BYTES or len(raw) > FRAME_MAX + FCS_BYTES:
        raise PacketError("frame_length")
    if fcs(raw[:-4]) != raw[-4:]:
        raise PacketError("fcs")
    return raw[:-4]


def state_text(state: int) -> str:
    parts = [name for bit, name in ((STATE_NET_READY, "NET_READY"), (STATE_MEAS_EN, "MEAS_EN"),
                                    (STATE_FAULT, "FAULT")) if state & bit]
    return "|".join(parts) if parts else "-"
