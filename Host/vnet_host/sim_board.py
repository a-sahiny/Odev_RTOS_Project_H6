"""Kartın protokol düzeyinde Python modeli (kart yokken host aracını ve testleri denemek için).

UYARI: Bu bir modeldir, kartın yerine geçmez. FreeRTOS zamanlaması, DMA, kesmeler,
gerçek IMU yoktur. Satır kuralları, frame kontrolleri, ret nedenleri, paket biçimi,
komutlar / dedup / SEQ, TX queue'su (8, son 2 yer kontrol cevaplarına, TELEM öne), HOLD /
TXDELAY enjeksiyonu, 1 Mbaud hat süresi ve her IMU örneğinde bir TELEM (SET_RATE hızında; roll /
pitch / yaw burada üretilen bir salınımdır, filtre yok; bir önceki TELEM'in DRDY -> TX bitti süresi)
firmware'e göre modellenmiştir. Sayaç adları
app_stats.h ile aynıdır; muhasebe eşitlikleri (tasarım 18.2) bu modelde de tutar.
"""

import collections
import json
import math
import random
import struct
import threading
import time

from . import ethframe as eth
from . import proto
from .crc import fcs_ok
from .stats_ids import COUNTER_NAMES

LINE_MAX = 540
LINE_IDLE_S = 0.200
TXQ_LEN = 8            # rapor 086: 6 -> 8
TXQ_RESERVE = 2
SB_LOW_BYTES = 2048
MB_LEN_BYTES = 4
REC_HDR = 12
LOW_QUEUE = 8          # workQueue
DEDUP_SLOTS = 8
DEDUP_WINDOW_S = 10.0
ARP_WAIT_S = 2.0


class SimBoard:
    def __init__(self, seed=1, baud=1000000, mac=bytes.fromhex("0280E1A1B2C3"), st_period=1.0,
                 boot_delay=0.15, realtime=True):
        self.rng = random.Random(seed)
        self.baud = baud
        self.char_s = 10.0 / baud
        self.mac = bytes(mac)
        self.ip = eth.BOARD_IP
        self.st_period = st_period
        self.realtime = realtime
        self.c = {n: 0 for n in COUNTER_NAMES}
        self.epoch = self.rng.randrange(1, 0x10000)
        self.t0 = time.perf_counter()
        self.net_ready_at = self.t0 + boot_delay
        self.meas_en = False
        self.fault = 0
        self.rate = proto.RATE_DEFAULT_HZ
        self.dest = None
        self.cal_t = None               # CAL uygulandığı an; 1 s sonra o duruş sıfır olur
        self.cal_zero = None            # (roll, pitch, yaw) sıfırı
        self.arp = {}                       # ip -> mac
        self.arp_pending = None             # (frame, t)
        self.dedup = collections.deque()    # (ip, req_id, type, payload, status, extra, t)
        self.reply_seq = 0
        self.telem_seq = 0
        self.seq_state = {}                 # (lane, kanal) -> [son, eski_sayısı]
        self.low_q = collections.deque()    # (bytes) sbLow
        self.low_bytes = 0
        self.hold_until = 0.0
        self.hold_pending = 0
        self.txdelay_ms = 0
        self.txq = collections.deque()
        self.env = {}                       # kanal -> [count, min, max, sum]
        self.last_telem = None              # UART'tan çıkan son TELEM: (SEQ, DRDY -> TX bitti µs)
        self.telem_front = None             # kuyruğun önündeki TELEM (rapor 092)
        self.telem_may_front = True         # arada başka satır gitti mi (rapor 092)
        self.meas_t0 = 0.0
        self.nb_min = 18
        self.heap_free = 40000
        # satır birleştirici
        self.line = bytearray()
        self.discard = False
        self.last_byte = time.perf_counter()
        self.out = None
        self._lock = threading.RLock()
        self._txcv = threading.Condition()
        self._running = True
        self._threads = [threading.Thread(target=self._tx_task, name="sim-uartTx", daemon=True),
                         threading.Thread(target=self._tick_task, name="sim-tick", daemon=True)]

    # ------------------------------------------------------------ bağlantı
    def attach(self, out_fn):
        self.out = out_fn
        for t in self._threads:
            t.start()
        self._report({"type": "boot", "ep": self.epoch, "fw": "0.4-sim"})

    def stop(self):
        self._running = False
        with self._txcv:
            self._txcv.notify_all()

    def now_tick(self):
        return int((time.perf_counter() - self.t0) * 1000) & 0xFFFFFFFF

    def state_bits(self):
        s = 0
        if time.perf_counter() >= self.net_ready_at:
            s |= proto.STATE_NET_READY
        if self.meas_en:
            s |= proto.STATE_MEAS_EN
        if self.fault:
            s |= proto.STATE_FAULT
        return s

    # ------------------------------------------------------------ TX yolu (uart_tx.c)
    def _admit(self, kind, data, is_ctrl, telem=None):
        """prvAdmit: boş yer 0 -> TXQ_FULL; kontrol dışı ve boş yer <= 2 -> TXQ_RESERVE.
        Yalnız TELEM queue'nun önüne (xQueueSendToFront); rezervi yalnız komut cevapları kullanır.
        telem: (DRDY zamanı, SEQ)."""
        with self._txcv:
            free = TXQ_LEN - len(self.txq)
            if free == 0:
                self.c["txq_full"] += 1
                return False
            to_front = telem is not None and self.telem_front is None and self.telem_may_front
            if not is_ctrl and free <= TXQ_RESERVE:
                self.c["txq_reserve"] += 1
                return False
            if to_front:   # yalnız TELEM, önde en çok bir, sırayla (rapor 092)
                self.telem_front = data
                self.telem_may_front = False
                self.txq.appendleft((kind, data, telem))
            else:
                self.txq.append((kind, data, telem))
            self.c["txq_ok"] += 1
            self._txcv.notify()
            return True

    def _post_frame(self, frame, is_ctrl=False, telem=None):
        self.c["if_out"] += 1
        return self._admit("frame", frame, is_ctrl, telem)

    def _report(self, obj):
        text = json.dumps(obj, separators=(",", ":"))
        if len(text) > 320:
            return
        self.c["tx_report_in"] += 1
        if not self._admit("report", text, False):
            self.c["pool_report_empty"] += 1

    def _tx_task(self):
        while self._running:
            with self._txcv:
                while not self.txq and self._running:
                    self._txcv.wait(0.2)
                if not self._running:
                    return
                kind, data, telem = self.txq.popleft()
                if data is self.telem_front:
                    self.telem_front = None
                else:
                    self.telem_may_front = True
            if kind == "frame":
                if len(data) < 60:
                    data = data + b"\x00" * (60 - len(data))
                from .lines import frame_line
                line = frame_line(data, 1)
            else:
                line = (data + "\n").encode()
            if self.realtime:
                time.sleep(len(line) * self.char_s)
            self.c["tx_lines"] += 1
            if telem is not None:
                # TELEM: DRDY -> TX bitti (uart_tx.c prvTelemDone), bir sonraki TELEM taşır
                t_drdy, seq = telem
                ms = (time.perf_counter() - t_drdy) * 1000.0
                self.last_telem = (seq, max(1, min(0xFFFF, int(ms * 1000))))
                if ms > proto.TELEM_TARGET_MS:
                    self.c["telem_over10"] += 1
                if ms > proto.TELEM_DEADLINE_MS:
                    self.c["telem_miss"] += 1
            if self.out is not None:
                self.out(line)
            if self.txdelay_ms:
                time.sleep(self.txdelay_ms / 1000.0)

    # ------------------------------------------------------------ RX: satırlar (line_rx.c + vnet_if.c)
    def feed(self, data: bytes):
        with self._lock:
            self.c["rx_bytes"] += len(data)
            for b in data:
                self.last_byte = time.perf_counter()
                if b == 0x0D:
                    continue
                if b == 0x0A:
                    if self.discard:
                        self.discard = False
                    elif self.line:
                        text = bytes(self.line)
                        self.line.clear()
                        self._on_line(text)
                    continue
                if self.discard:
                    continue
                if len(self.line) >= LINE_MAX:
                    self.line.clear()
                    self.discard = True
                    self._reject("line_too_long")
                    continue
                self.line.append(b)

    def _poll_line(self):
        with self._lock:
            if time.perf_counter() - self.last_byte >= LINE_IDLE_S:
                if self.discard:
                    self.discard = False
                elif self.line:
                    self.line.clear()
                    self._reject("line_timeout")

    def _reject(self, counter):
        self.c["lines_total"] += 1 if counter.startswith("line_") else 0
        self.c[counter] += 1
        self._report({"type": "rx_err", "reason": counter})

    def _on_line(self, text: bytes):
        s = text.decode("latin-1")
        if s in ("HELLO", "STATE") or s.startswith("HOLD ") or s.startswith("TXDELAY "):
            self._console(s)
            return
        if not s.startswith("FRAME "):
            self.c["lines_total"] += 1
            self.c["line_unknown"] += 1
            self._report({"type": "error", "reason": "unknown_command"})
            return
        rest = s[6:]
        flag = None
        if len(rest) >= 2 and rest[1] == " ":
            if rest[0] not in "01":
                self._reject("line_bad_flag")
                return
            flag = int(rest[0])
            rest = rest[2:]
        if not rest or len(rest) & 1 or any(ch not in "0123456789abcdefABCDEF" for ch in rest):
            self._reject("line_bad_hex")
            return
        raw = bytes.fromhex(rest)
        self.c["lines_total"] += 1
        self.c["frames"] += 1
        self.c[{None: "rx_fcs_none", 0: "rx_fcs0", 1: "rx_fcs1"}[flag]] += 1
        n = len(raw) - (4 if flag == 1 else 0)
        if not 60 <= n <= 256:
            self.c["frame_length"] += 1
            self._report({"type": "rx_err", "reason": "frame_length"})
            return
        if flag == 1:
            if not fcs_ok(raw):
                self.c["frame_fcs"] += 1
                self._report({"type": "rx_err", "reason": "frame_fcs"})
                return
            raw = raw[:-4]
        f = eth.parse_frame(raw, self.mac)
        for reason in ("frame_ethertype", "frame_dst_mac", "ip_csum", "udp_csum"):
            if reason in f.errors:
                self.c[reason] += 1
                self._report({"type": "rx_err", "reason": reason})
                return
        if f.errors:                       # ip_length vb.: kart bunları ip_csum sayar
            self.c["ip_csum"] += 1
            self._report({"type": "rx_err", "reason": "ip_csum"})
            return
        self.c["to_stack"] += 1
        if f.kind == "UDP" and f.dport == proto.PORT_CTRL:
            self.c["to_stack_crit"] += 1
        elif f.kind == "UDP" and f.dport == proto.PORT_LOW:
            self.c["to_stack_low"] += 1
        self.c["ip_evt_rx"] += 1
        self._stack_rx(f)

    def _console(self, s):
        self.c["lines_total"] += 1
        if s == "HELLO":
            self.c["console_cmds"] += 1
            self._report({"type": "hello", "fw": "0.4-sim", "board": "STM32F746G-DISCO (sim)",
                          "ip": self.ip, "ports": [5000, 5001, 5002], "ep": self.epoch})
        elif s == "STATE":
            self.c["console_cmds"] += 1
            self._report(self._resources("state"))
        else:
            key, _, arg = s.partition(" ")
            limit = 10000 if key == "HOLD" else 1000
            if not arg.isdigit() or int(arg) > limit:
                self.c["line_bad_arg"] += 1
                self._report({"type": "error", "reason": "bad_arg"})
                return
            self.c["console_cmds"] += 1
            if key == "HOLD":
                self.hold_pending = int(arg)
                self._report({"type": "ack", "result": "hold_requested"})
            else:
                self.txdelay_ms = int(arg)
                self._report({"type": "ack", "result": "txdelay_set"})

    # ------------------------------------------------------------ IP task
    def _stack_rx(self, f):
        if f.kind == "ARP":
            if f.arp_sender_ip:
                self.arp[f.arp_sender_ip] = f.arp_sender_mac
            if f.arp_op == 1 and f.arp_target_ip == self.ip:
                self._post_frame(eth.build_arp(2, self.mac, self.ip, f.arp_sender_mac, f.arp_sender_ip))
            if self.arp_pending is not None and f.arp_sender_ip in self.arp:
                pend, self.arp_pending = self.arp_pending, None
                self._stack_rx(pend[0])
            return
        if f.ip_dst != self.ip:
            return
        if f.kind == "ICMP" and f.icmp_type == 8:
            if f.ip_src not in self.arp:
                self.arp[f.ip_src] = f.src     # yanıt için (stack ARP cache'inden okur)
            ip_hdr = f.raw[14:34]
            icmp = bytearray(f.raw[34:14 + struct.unpack(">H", ip_hdr[2:4])[0]])
            icmp[0] = 0
            icmp[2:4] = b"\x00\x00"
            icmp[2:4] = struct.pack(">H", eth.checksum16(bytes(icmp)))
            ip = eth.ipv4_header(self.ip, f.ip_src, eth.IP_PROTO_ICMP, len(icmp), self._ip_id())
            self._post_frame(eth.eth_header(self.arp[f.ip_src], self.mac, eth.ETHERTYPE_IPV4) + ip + bytes(icmp))
            return
        if f.kind != "UDP":
            return
        if f.ip_src not in self.arp:
            # FreeRTOS_UDP_IPv4.c:388-395: tek paket bekletilir, ARP isteği gider
            if self.arp_pending is None:
                self.arp_pending = (f, time.perf_counter())
                self.c["arp_delayed"] += 1
                self._arp_request(f.ip_src)
            else:
                self.c["arp_delayed_full"] += 1
            return
        if f.dport == proto.PORT_CTRL:
            self._callback("crit", f)
        elif f.dport == proto.PORT_LOW:
            self._callback("low", f)

    def _ip_id(self):
        self._ipid = (getattr(self, "_ipid", 0) + 1) & 0xFFFF
        return self._ipid

    def _arp_request(self, ip):
        self._post_frame(eth.build_arp(1, self.mac, self.ip, b"\x00" * 6, ip))

    def _sendto(self, sport, dst_ip, dport, payload, is_ctrl, telem=None):
        """FreeRTOS_sendto: hedef ARP'de yoksa paket ARP isteğine dönüşür (payload kaybolur)."""
        mac = self.arp.get(dst_ip)
        if mac is None:
            self.c["arp_tx_dropped"] += 1
            self._arp_request(dst_ip)
            return True                       # sendto yine > 0 döner (tasarım 10.3)
        frame = eth.build_udp_frame(mac, self.mac, self.ip, dst_ip, sport, dport, payload, self._ip_id())
        self.c["ip_evt_tx"] += 1
        return self._post_frame(frame, is_ctrl, telem) or True

    # ------------------------------------------------------------ udp_in.c callback
    def _callback(self, lane, f):
        n = len(f.udp_payload)
        self.c["cb_" + lane] += 1
        if n > proto.MAX_PACKET:            # boş datagram yazılır, parser PKT_SHORT der (rapor 062)
            self.c["dgram_bad_len_" + lane] += 1
            return
        if lane == "crit":
            self.c["sb_written_crit"] += 1
            self._parse("crit", f.ip_src, f.sport, f.udp_payload, time.perf_counter())
            return
        need = MB_LEN_BYTES + REC_HDR + n
        if self.low_bytes + need > SB_LOW_BYTES:
            self.c["cb_low_sb_full"] += 1
            return
        self.c["sb_written_low"] += 1
        self.low_q.append((f.ip_src, f.sport, f.udp_payload, need))
        self.low_bytes += need

    # ------------------------------------------------------------ pkt_parser.c
    ROUTES = {
        "crit": {proto.CMD_START: (0, 0, 0, 0), proto.CMD_STOP: (0, 0, 0, 0),
                 proto.CMD_SET_RATE: (2, 2, 0, 0), proto.CMD_GET_STATS: (1, 1, 0, 0)},
        "low": {proto.ENV: (3, 17, 2, 0), proto.BULK: (5, 200, 0, 1), proto.CAL: (12, 12, 0, 2)},
    }

    def _parse(self, lane, src_ip, sport, data, t_rx):
        sfx = "_" + lane
        self.c["rec_read" + sfx] += 1
        try:
            pkt = proto.decode(data)
        except proto.PacketError as e:
            self.c["pkt_%s%s" % (e.reason, sfx)] += 1
            return
        if pkt.is_response:
            self.c["pkt_flags" + sfx] += 1
            return
        route = self.ROUTES[lane].get(pkt.type)
        ln = len(pkt.payload)
        if lane == "crit":
            # Komut portu (pkt_parser.c prvRoute, rapor 085): başka lane'in türü ya da TELEM
            # cevapsız düşer; gerisi komuttur. Bilinmeyen türü ve yanlış LEN'i cmd.c cevaplar
            # (E_TYPE / E_LEN), parser bunları saymaz.
            if route is None and (pkt.type in self.ROUTES["low"] or pkt.type == proto.TELEM):
                self.c["pkt_wrong_port" + sfx] += 1
                return
            chan = 0
        else:
            if route is None:
                self.c["pkt_wrong_port" + sfx] += 1
                return
            lo, hi, stride, chan = route
            if ln < lo or ln > hi or (stride and (pkt.payload[0] == 0 or pkt.payload[0] > proto.ENV_MAX_CHANNELS
                                                  or ln != 1 + stride * pkt.payload[0])):
                self.c["pkt_type_len" + sfx] += 1
                return
        self.c["pkt_ok" + sfx] += 1
        if not self._track_seq(lane, chan, pkt.seq) and lane == "low":
            return
        self.c["evt_posted" + sfx] += 1
        if lane == "crit":
            self.c["evt_handled_ctrl"] += 1
            self._cmd(src_ip, sport, pkt, t_rx)
        else:
            self.c["evt_handled_work"] += 1
            self._work(pkt)

    def _track_seq(self, lane, chan, seq):
        sfx = "_" + lane
        st = self.seq_state.get((lane, chan))
        if st is None:
            self.seq_state[(lane, chan)] = [seq, 0]
            return True
        d = (seq - st[0]) & 0xFFFF
        if d == 0:
            self.c["seq_dup" + sfx] += 1
            return lane == "crit"
        if d < 0x8000:
            self.c["seq_gap" + sfx] += d - 1
            st[0], st[1] = seq, 0
            return True
        st[1] += 1
        if st[1] >= 3:
            self.c["seq_resync" + sfx] += 1
            st[0], st[1] = seq, 0
            return True
        self.c["seq_old" + sfx] += 1
        return lane == "crit"

    # ------------------------------------------------------------ cmd.c
    def _cmd(self, src_ip, sport, pkt, t_rx):
        now = time.perf_counter()
        while self.dedup and (now - self.dedup[0][6] > DEDUP_WINDOW_S):
            self.dedup.popleft()
        extra = b""
        if pkt.req_id == 0:
            status = proto.ST_E_REQID
            self.c["cmd_invalid"] += 1
            extra = self._type_extra(pkt, status)
        else:
            seen = next((d for d in self.dedup if d[0] == src_ip and d[1] == pkt.req_id), None)
            # İçerik = tür + LEN + payload'un ilk 4 baytı (cmd.c prvKeptBytes; geçerli her
            # komutun payload'u ≤ 2 bayt olduğu için onlarda tamamı).
            if seen is not None and (seen[2], len(seen[3]), seen[3][:4]) == \
                    (pkt.type, len(pkt.payload), pkt.payload[:4]):
                status = seen[4]
                self.c["cmd_replay"] += 1
                extra = seen[5] if seen[5] else self._type_extra(pkt, status)
            elif seen is not None:
                status = proto.ST_E_CONFLICT
                self.c["cmd_conflict"] += 1
                extra = self._type_extra(pkt, status)
            else:
                status = self._validate(pkt)
                if status == proto.ST_OK:
                    self._execute(pkt, src_ip, sport)
                    self.c["cmd_new"] += 1
                else:
                    self.c["cmd_invalid"] += 1
                extra = self._type_extra(pkt, status)
                cache = extra if pkt.type != proto.CMD_GET_STATS else b""
                self.dedup.append((src_ip, pkt.req_id, pkt.type, pkt.payload, status, cache, now))
                while len(self.dedup) > DEDUP_SLOTS:
                    self.dedup.popleft()
        lat = int((time.perf_counter() - t_rx) * 1e6) + 60
        self.reply_seq = (self.reply_seq + 1) & 0xFFFF
        flags = proto.FLAG_RESP | (proto.FLAG_ERR if status != proto.ST_OK else 0)
        body = proto.payload_response(status, self.epoch, self.state_bits(), lat, extra)
        data = proto.encode(proto.Packet(pkt.type, flags, self.reply_seq, pkt.req_id, body))
        self._sendto(proto.PORT_CTRL, src_ip, sport, data, True)

    def _validate(self, pkt):
        """cmd.c prvValidate sırası: tür -> uzunluk -> aralık / durum (tasarım 16.2, 16.3)."""
        ln = len(pkt.payload)
        if pkt.type == proto.CMD_START:
            if ln != 0:
                return proto.ST_E_LEN
            if not self.state_bits() & proto.STATE_NET_READY:
                return proto.ST_E_STATE
            if self.fault and self._active_faults():
                return proto.ST_E_STATE
        elif pkt.type == proto.CMD_STOP:
            if ln != 0:
                return proto.ST_E_LEN
        elif pkt.type == proto.CMD_SET_RATE:
            if ln != 2:
                return proto.ST_E_LEN
            hz = struct.unpack(">H", pkt.payload)[0]
            if hz not in proto.RATES_HZ:
                return proto.ST_E_RANGE
        elif pkt.type == proto.CMD_GET_STATS:
            if ln != 1:
                return proto.ST_E_LEN
            if pkt.payload[0] >= proto.STATS_PAGES:
                return proto.ST_E_RANGE
        else:
            return proto.ST_E_TYPE
        return proto.ST_OK

    def _active_faults(self):
        return 0

    def _execute(self, pkt, src_ip, sport):
        if pkt.type == proto.CMD_START:
            if self.fault:
                self.fault = 0
                self._report({"type": "fault", "set": 0, "why": 0})
            self.dest = (src_ip, sport)
            self.meas_t0 = time.perf_counter()
            self.meas_en = True
        elif pkt.type == proto.CMD_STOP:
            self.meas_en = False
        elif pkt.type == proto.CMD_SET_RATE:
            self.rate = struct.unpack(">H", pkt.payload)[0]

    def _type_extra(self, pkt, status):
        if pkt.type == proto.CMD_SET_RATE:
            return struct.pack(">H", self.rate)
        if pkt.type == proto.CMD_GET_STATS and status == proto.ST_OK:
            page = pkt.payload[0]
            tick = self.now_tick()
            n = proto.COUNTERS_PER_PAGE
            if page < 3:
                # rapor 092 (cmd.c): sayfa 0 görüntüyü alır, 1 ve 2 aynı görüntüden ve aynı tick'le
                if page == 0 or getattr(self, "_stats_snap", None) is None:
                    self._stats_snap = (tick, [self.c[name] & 0xFFFFFFFF for name in COUNTER_NAMES])
                tick, snap = self._stats_snap
                vals = snap[page * n:page * n + n]
            else:
                vals = []
                for ch in sorted(self.env)[:8]:
                    cnt, mn, mx, sm = self.env[ch]
                    mean = int(sm / cnt) if cnt else 0
                    vals += [cnt, mn & 0xFFFFFFFF, mx & 0xFFFFFFFF, mean & 0xFFFFFFFF]
            return bytes([page]) + struct.pack(">I", tick) + bytes([len(vals)]) + \
                struct.pack(">%dI" % len(vals), *vals)
        return b""

    # ------------------------------------------------------------ worker_task.c
    def _work(self, pkt):
        p = pkt.payload
        if pkt.type == proto.ENV:
            n = p[0]
            vals = struct.unpack(">%dh" % n, p[1:1 + 2 * n])
            for ch, v in enumerate(vals):
                e = self.env.setdefault(ch, [0, v, v, 0])
                e[0] += 1
                e[1] = min(e[1], v)
                e[2] = max(e[2], v)
                e[3] += v
            self.c["env_pkts"] += 1
        elif pkt.type == proto.BULK:
            seed = struct.unpack(">I", p[:4])[0]
            m = len(p) - 4
            self.c["bulk_pkts"] += 1
            self.c["bulk_bytes"] += m
            if p[4:] != proto.bulk_pattern(seed, m):
                self.c["bulk_bad"] += 1
        elif pkt.type == proto.CAL:
            vals = struct.unpack(">6h", p)
            if all(abs(v) <= proto.CAL_LIMIT for v in vals):
                self.cal_t, self.cal_zero = time.perf_counter(), None
                self.c["cal_applied"] += 1
            else:
                self.c["cal_rejected"] += 1

    def _low_parser(self):
        """lowParser P1: her record'dan önce HOLD; ardından en çok workQueue kadar işle."""
        now = time.perf_counter()
        if self.hold_pending:
            self.hold_until = now + self.hold_pending / 1000.0
            self.hold_pending = 0
        if now < self.hold_until:
            return
        for _ in range(LOW_QUEUE):
            if not self.low_q:
                return
            ip, sport, data, need = self.low_q.popleft()
            self.low_bytes -= need
            self._parse("low", ip, sport, data, now)

    # ------------------------------------------------------------ meas_task.c
    def _attitude(self, t):
        """Bir uçuş: yavaş roll / pitch salınımı, sabit hızda yaw (t: START'tan beri s). İvme açılarla
        tutarlı (yer çekimi gövde ekseninde), jiroskop açıların türevi. İlk 1 s bias ölçümü: yaw 0,
        BIAS_READY yok (meas_task.c ile aynı davranış). CAL: 1 s sonra o anki duruş sıfır sayılır (kart durmayı
        bekler; simülatör hep uçar, beklemez)."""
        roll = 35.0 * math.sin(2 * math.pi * 0.10 * t)
        pitch = 20.0 * math.sin(2 * math.pi * 0.07 * t + 0.6)
        bias = t < 1.0
        yaw = 0.0 if bias else ((25.0 * (t - 1.0) + 180.0) % 360.0) - 180.0
        flags = proto.ATT_FLAG_ACC_USED | (0 if bias else proto.ATT_FLAG_BIAS_READY)
        if self.cal_t is not None and self.cal_zero is None:
            flags |= proto.ATT_FLAG_CAL_BUSY
            if time.perf_counter() - self.cal_t >= 1.0:
                self.cal_zero = (roll, pitch, yaw)
        if self.cal_zero is not None:
            flags = (flags | proto.ATT_FLAG_CAL_DONE) & ~proto.ATT_FLAG_CAL_BUSY
            roll, pitch = roll - self.cal_zero[0], pitch - self.cal_zero[1]
            yaw = (yaw - self.cal_zero[2] + 180.0) % 360.0 - 180.0
        r, p = math.radians(roll), math.radians(pitch)
        g = 8192
        accel = (int(-g * math.sin(p)), int(g * math.cos(p) * math.sin(r)), int(g * math.cos(p) * math.cos(r)))
        droll = 35.0 * 2 * math.pi * 0.10 * math.cos(2 * math.pi * 0.10 * t)
        dpitch = 20.0 * 2 * math.pi * 0.07 * math.cos(2 * math.pi * 0.07 * t + 0.6)
        gyro = tuple(int(v * proto.GYRO_LSB_PER_DPS) for v in (droll, dpitch, 0.0 if bias else 25.0))
        clamp = lambda v: max(-32768, min(32767, v))
        accel = tuple(clamp(v) for v in accel)
        gyro = tuple(clamp(v) for v in gyro)
        ang = tuple(max(-18000, min(18000, int(round(a * 100)))) for a in (roll, pitch, yaw))
        return accel, gyro, ang, flags

    def _meas_step(self):
        """Bir IMU örneği (DRDY) = bir TELEM; önceki TELEM'in kart içi gecikmesi bu mesajda gider."""
        if not (self.meas_en and self.state_bits() & proto.STATE_NET_READY) or self.dest is None:
            return
        t_drdy = time.perf_counter()
        accel, gyro, ang, flags = self._attitude(t_drdy - self.meas_t0)
        prev_seq, prev_us = self.last_telem if self.last_telem is not None else (0, 0)
        if self.last_telem is not None:
            flags |= proto.TELEM_FLAG_PREV
        payload = proto.payload_telem(accel, gyro, ang, flags, prev_seq, prev_us)
        seq = self.telem_seq
        data = proto.encode(proto.Packet(proto.TELEM, 0, seq, 0, payload))
        self.telem_seq = (self.telem_seq + 1) & 0xFFFF
        self.c["telem_gen"] += 1
        self.c["telem_sendto_ok"] += 1
        self._sendto(proto.PORT_TELEM, self.dest[0], self.dest[1], data, False, (t_drdy, seq))

    # ------------------------------------------------------------ periyodik işler
    def _resources(self, typ):
        hold_left = max(0, int((self.hold_until - time.perf_counter()) * 1000))
        mask = (1 if hold_left or self.hold_pending else 0) | (2 if self.txdelay_ms else 0)
        with self._txcv:
            q_free = TXQ_LEN - len(self.txq)
        return {"type": typ, "t": self.now_tick(), "ep": self.epoch, "s": self.state_bits(), "why": self.fault,
                "inj": [mask, hold_left or self.hold_pending, self.txdelay_ms], "nb": [18, self.nb_min],
                "ipq": 23, "cp": [4, 4], "lp": [8, 8], "rp": [2, 1], "q": [6, 8, q_free],
                "sb": [0, 0, self.low_bytes], "lag": 0, "heap": [self.heap_free, self.heap_free],
                "hwm": [180, 300, 200, 150, 190, 180, 210, 300, 60]}

    def _tick_task(self):
        next_meas = time.perf_counter()
        next_st = time.perf_counter() + self.st_period
        while self._running:
            now = time.perf_counter()
            with self._lock:
                self._poll_line()
                if self.arp_pending is not None and now - self.arp_pending[1] > ARP_WAIT_S:
                    self.arp_pending = None
                if now >= next_meas:
                    next_meas += 1.0 / self.rate
                    if now - next_meas > 0.2:
                        next_meas = now
                    self._meas_step()
                self._low_parser()
                if self.st_period and now >= next_st:
                    next_st += self.st_period
                    self._report(self._resources("st"))
                    chans = [[e[0], e[1], e[2], int(e[3] / e[0]) if e[0] else 0]
                             for _, e in sorted(self.env.items())[:8]]
                    self._report({"type": "env", "t": self.now_tick(), "ch": chans})
            time.sleep(0.002)
