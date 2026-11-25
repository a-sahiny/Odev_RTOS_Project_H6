"""Uygulama paketi (tasarım 15, 16). Kartın pkt_format.c / cmd.c ile aynı biçim.

    off  boy  alan
    0    1    SOF    0xA5
    1    1    VER    0x01
    2    1    TYPE
    3    1    FLAGS  bit0 RESP, bit1 ERR
    4    2    SEQ    big-endian
    6    2    REQ_ID big-endian (komutlarda 1..65535, veride 0)
    8    1    LEN    0..200
    9    LEN  PAYLOAD
    9+L  1    CRC8   bayt 0..8+LEN üzerinden
"""

from dataclasses import dataclass, field
import struct

from .crc import crc8

SOF = 0xA5
VERSION = 0x01
HEADER_BYTES = 9
OVERHEAD_BYTES = 10
MAX_PAYLOAD = 200
MAX_PACKET = OVERHEAD_BYTES + MAX_PAYLOAD

FLAG_RESP = 0x01
FLAG_ERR = 0x02
FLAG_KNOWN = FLAG_RESP | FLAG_ERR

# TYPE
CMD_START = 0x01
CMD_STOP = 0x02
CMD_SET_RATE = 0x03
CMD_GET_STATS = 0x04
# 0x05 GET_IMU'ydu: açılar artık her IMU örneğinde TELEM ile gelir (kart cevabı E_TYPE).
ENV = 0x20
BULK = 0x21
CAL = 0x22
TELEM = 0x30

TYPE_NAMES = {
    CMD_START: "START", CMD_STOP: "STOP", CMD_SET_RATE: "SET_RATE", CMD_GET_STATS: "GET_STATS",
    ENV: "ENV", BULK: "BULK", CAL: "CAL", TELEM: "TELEM",
}
COMMAND_TYPES = (CMD_START, CMD_STOP, CMD_SET_RATE, CMD_GET_STATS)
LOW_TYPES = (ENV, BULK, CAL)

# UDP portları (kart tarafı)
PORT_CTRL = 5000
PORT_LOW = 5001
PORT_TELEM = 5002

# STATUS (tasarım 16.2)
ST_OK = 0x00
ST_E_LEN = 0x01
ST_E_RANGE = 0x02
ST_E_STATE = 0x03
ST_E_CONFLICT = 0x04
ST_E_TYPE = 0x05
ST_E_REQID = 0x06
STATUS_NAMES = {
    ST_OK: "OK", ST_E_LEN: "E_LEN", ST_E_RANGE: "E_RANGE", ST_E_STATE: "E_STATE",
    ST_E_CONFLICT: "E_CONFLICT", ST_E_TYPE: "E_TYPE", ST_E_REQID: "E_REQID",
}

# STATE baytı (cevaplarda ve JSON "s" alanında)
STATE_NET_READY = 0x01
STATE_MEAS_EN = 0x02
STATE_FAULT = 0x04

# FAULT nedeni bitleri (JSON "why")
FAULT_NAMES = {0x01: "IMU", 0x02: "POOL", 0x04: "UART_ORE", 0x08: "UART_TX_STALL", 0x10: "HEAP"}

# SET_RATE = IMU örnekleme hızı = TELEM hızı; kart yalnız bunları kabul eder (değilse E_RANGE).
RATES_HZ = (10, 20, 50, 100, 200)
RATE_MIN_HZ = 10
RATE_MAX_HZ = 200
RATE_DEFAULT_HZ = 100
STATS_PAGES = 4               # 0..2 sayaçlar, 3 ENV
COUNTERS_PER_PAGE = 43       # 127 sayaç: 43 / 43 / 41
ENV_MAX_CHANNELS = 8
BULK_MAX_PATTERN = 196
CAL_LIMIT = 4096              # worker: |offset| <= 4096
TELEM_LEN = 23
TELEM_M33_LEN = TELEM_LEN + 2  # MP257 yolu: + önceki paketin M33 süresi (T1 -> T5, µs)
TELEM_TARGET_MS = 10.0        # kart içi DRDY -> TX bitti hedefi
TELEM_DEADLINE_MS = 12.0      # kart içi DRDY -> TX bitti deadline'ı
ATT_FLAG_BIAS_READY = 0x01
ATT_FLAG_ACC_USED = 0x02
TELEM_FLAG_PREV = 0x04        # prev_seq / prev_us geçerli
DEDUP_SLOTS = 8
DEDUP_WINDOW_S = 10.0


def state_text(state: int) -> str:
    parts = []
    if state & STATE_NET_READY:
        parts.append("NET_READY")
    if state & STATE_MEAS_EN:
        parts.append("MEAS_EN")
    if state & STATE_FAULT:
        parts.append("FAULT")
    return "|".join(parts) if parts else "-"


def fault_text(why: int) -> str:
    names = [name for bit, name in FAULT_NAMES.items() if why & bit]
    return "|".join(names) if names else "-"


@dataclass
class Packet:
    type: int
    flags: int = 0
    seq: int = 0
    req_id: int = 0
    payload: bytes = b""

    @property
    def is_response(self) -> bool:
        return bool(self.flags & FLAG_RESP)

    @property
    def name(self) -> str:
        return TYPE_NAMES.get(self.type, "0x%02X" % self.type)


class PacketError(ValueError):
    """reason: kartın PKT_* sayaçlarıyla aynı adlar (short, sof, version, length, crc, flags)."""

    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


def encode(pkt: Packet) -> bytes:
    if len(pkt.payload) > MAX_PAYLOAD:
        raise ValueError("payload > 200")
    if pkt.flags & ~FLAG_KNOWN:
        raise ValueError("reserved flag")
    head = struct.pack(">BBBBHHB", SOF, VERSION, pkt.type & 0xFF, pkt.flags, pkt.seq & 0xFFFF,
                       pkt.req_id & 0xFFFF, len(pkt.payload))
    body = head + bytes(pkt.payload)
    return body + bytes([crc8(body)])


def encode_raw(type_: int, flags: int, seq: int, req_id: int, payload: bytes,
               length: int = None, crc: int = None, sof: int = SOF, version: int = VERSION) -> bytes:
    """Bozuk paket üretmek için (T02): her alan elle verilebilir."""
    ln = len(payload) if length is None else length
    head = struct.pack(">BBBBHHB", sof, version, type_ & 0xFF, flags & 0xFF, seq & 0xFFFF, req_id & 0xFFFF, ln & 0xFF)
    body = head + bytes(payload)
    return body + bytes([crc8(body) if crc is None else crc])


def decode(data: bytes) -> Packet:
    """Kartın Pkt_Decode sırası: boy -> SOF -> VER -> LEN -> CRC -> FLAGS."""
    if len(data) < OVERHEAD_BYTES:
        raise PacketError("short")
    if data[0] != SOF:
        raise PacketError("sof")
    if data[1] != VERSION:
        raise PacketError("version")
    ln = data[8]
    if ln > MAX_PAYLOAD or len(data) != OVERHEAD_BYTES + ln:
        raise PacketError("length")
    if crc8(data[:-1]) != data[-1]:
        raise PacketError("crc")
    if data[3] & ~FLAG_KNOWN:
        raise PacketError("flags")
    seq, req_id = struct.unpack(">HH", data[4:8])
    return Packet(type=data[2], flags=data[3], seq=seq, req_id=req_id, payload=bytes(data[9:9 + ln]))


# ---------------------------------------------------------------- yükler
def payload_set_rate(hz: int) -> bytes:
    return struct.pack(">H", hz & 0xFFFF)


def payload_get_stats(page: int) -> bytes:
    return bytes([page & 0xFF])


def payload_env(values) -> bytes:
    values = list(values)
    if not 1 <= len(values) <= ENV_MAX_CHANNELS:
        raise ValueError("ENV: 1..8 kanal")
    return bytes([len(values)]) + b"".join(struct.pack(">h", int(v)) for v in values)


def bulk_pattern(seed: int, m: int) -> bytes:
    return bytes(((seed + i) & 0xFF) for i in range(m))


def payload_bulk(seed: int, m: int) -> bytes:
    if not 1 <= m <= BULK_MAX_PATTERN:
        raise ValueError("BULK: 1..196 bayt")
    return struct.pack(">I", seed & 0xFFFFFFFF) + bulk_pattern(seed, m)


def payload_cal(offsets) -> bytes:
    offsets = list(offsets)
    if len(offsets) != 6:
        raise ValueError("CAL: 6 offset (ax ay az gx gy gz)")
    return b"".join(struct.pack(">h", int(v)) for v in offsets)


# ---------------------------------------------------------------- cevaplar
@dataclass
class Response:
    """Komut cevabı (tasarım 15.3)."""
    type: int
    seq: int
    req_id: int
    flags: int
    status: int
    epoch: int
    state: int
    lat_us: int
    extra: bytes = b""
    rx_time: float = 0.0          # host saatiyle geliş (time.perf_counter)
    rtt_ms: float = 0.0           # gönderimden cevaba (host saati)
    attempts: int = 1

    @property
    def ok(self) -> bool:
        return self.status == ST_OK

    @property
    def status_name(self) -> str:
        return STATUS_NAMES.get(self.status, "0x%02X" % self.status)

    @property
    def applied_rate(self):
        if self.type == CMD_SET_RATE and len(self.extra) >= 2:
            return struct.unpack(">H", self.extra[:2])[0]
        return None


def parse_response(pkt: Packet) -> Response:
    p = pkt.payload
    if len(p) < 6:
        raise PacketError("resp_short")
    epoch = struct.unpack(">H", p[1:3])[0]
    lat = struct.unpack(">H", p[4:6])[0]
    return Response(type=pkt.type, seq=pkt.seq, req_id=pkt.req_id, flags=pkt.flags,
                    status=p[0], epoch=epoch, state=p[3], lat_us=lat, extra=bytes(p[6:]))


@dataclass
class StatsPage:
    page: int
    tick: int
    values: list = field(default_factory=list)


def parse_stats_page(extra: bytes) -> StatsPage:
    if len(extra) < 6:
        raise PacketError("stats_short")
    page = extra[0]
    tick = struct.unpack(">I", extra[1:5])[0]
    n = extra[5]
    if len(extra) != 6 + 4 * n:
        raise PacketError("stats_len")
    vals = list(struct.unpack(">%dI" % n, extra[6:6 + 4 * n]))
    return StatsPage(page=page, tick=tick, values=vals)


def env_channels(page: StatsPage):
    """Sayfa 3: kanal başına (count, min, max, mean); i16 değerler 32-bit işaret genişletilmiş."""
    out = []
    v = page.values
    for i in range(0, len(v) - 3, 4):
        def s32(x):
            return x - (1 << 32) if x & 0x80000000 else x
        out.append({"count": v[i], "min": s32(v[i + 1]), "max": s32(v[i + 2]), "mean": s32(v[i + 3])})
    return out


# ---------------------------------------------------------------- telemetri
ACCEL_LSB_PER_G = 8192.0      # ±4 g
GYRO_LSB_PER_DPS = 32.8       # ±1000 dps


@dataclass
class Telem:
    """TELEM (0x30, port 5002): her IMU örneğinde bir tane, 23 B, big-endian.

    ax ay az i16, gx gy gz i16 (ham LSB, CAL uygulanmış), roll pitch yaw i16 (0,01°), flags u8,
    prev_seq u16 + prev_us u16: bu mesajdan önce UART'tan çıkmış son TELEM'in SEQ'i ve onun kartta
    ölçülen DRDY -> TX bitti süresi (IMU verisi hazır -> satırın son biti UART'tan çıktı; PC / USB
    yok). Bir mesajın kendi süresi ancak o mesaj gittikten sonra bilinir, bu yüzden bir sonrakinde
    gelir. Örnek sırası başlıktaki SEQ'tir."""
    seq: int
    accel: tuple
    gyro: tuple
    roll: float          # derece
    pitch: float
    yaw: float
    flags: int
    prev_seq: int = 0
    prev_us: int = 0
    m33_us: int = 0      # MP257 yolu: önceki paketin M33 süresi, µs; 0 yok
    rx_time: float = 0.0
    linux: bool = False  # MP257'den (UDP) geldi

    @property
    def prev_valid(self) -> bool:
        return bool(self.flags & TELEM_FLAG_PREV)

    @property
    def prev_ms(self):
        return self.prev_us / 1000.0 if self.prev_valid else None

    @property
    def bias_ready(self) -> bool:
        return bool(self.flags & ATT_FLAG_BIAS_READY)

    @property
    def acc_used(self) -> bool:
        return bool(self.flags & ATT_FLAG_ACC_USED)

    @property
    def accel_g(self):
        return tuple(v / ACCEL_LSB_PER_G for v in self.accel)

    @property
    def gyro_dps(self):
        return tuple(v / GYRO_LSB_PER_DPS for v in self.gyro)


def parse_telem(pkt: Packet) -> Telem:
    if pkt.type != TELEM or len(pkt.payload) not in (TELEM_LEN, TELEM_M33_LEN):
        raise PacketError("telem_len")
    ax, ay, az, gx, gy, gz, r, p, y, flags, prev_seq, prev_us = struct.unpack(">hhhhhhhhhBHH", pkt.payload[:TELEM_LEN])
    m33_us = struct.unpack(">H", pkt.payload[TELEM_LEN:])[0] if len(pkt.payload) == TELEM_M33_LEN else 0
    return Telem(seq=pkt.seq, accel=(ax, ay, az), gyro=(gx, gy, gz), roll=r / 100.0, pitch=p / 100.0,
                 yaw=y / 100.0, flags=flags, prev_seq=prev_seq, prev_us=prev_us, m33_us=m33_us)


def payload_telem(accel, gyro, angles_cdeg, flags, prev_seq=0, prev_us=0) -> bytes:
    return struct.pack(">hhhhhhhhhBHH", *accel, *gyro, *angles_cdeg, flags & 0xFF, prev_seq & 0xFFFF,
                       min(prev_us, 0xFFFF))


def payload_response(status: int, epoch: int, state: int, lat_us: int, extra: bytes = b"") -> bytes:
    return bytes([status]) + struct.pack(">H", epoch & 0xFFFF) + bytes([state & 0xFF]) + \
        struct.pack(">H", min(lat_us, 0xFFFF)) + extra
