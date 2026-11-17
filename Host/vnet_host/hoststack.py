"""Host tarafı ağ yığını (tasarım 2, 11, 13.2, 16, 18.2 L8).

- Karttan gelen her FRAME 1 satırını doğrular: FCS, Ethernet, IPv4 başlık checksum,
  UDP/ICMP checksum, portlar, uygulama paketi (CRC8), REQ_ID eşlemesi.
- 192.168.50.1 için ARP isteklerine cevap verir (arp_mute ile kesilebilir, T07).
- Kartın MAC'ini ARP ile öğrenir; ping atar.
- Komut gönderir, cevabı REQ_ID ile eşler, zaman aşımında aynı REQ_ID ile tekrarlar.
- Telemetriyi alır, SEQ boşluklarını sayar.
- JSON raporları toplar (boot, hello, state, st, env, ack, error, rx_err, fault, halt).
"""

import collections
import contextlib
import itertools
import os
import threading
import time

from . import ethframe as eth
from . import lines
from . import proto
from .stats_ids import COUNTER_NAMES


class HostError(RuntimeError):
    pass


class StatusError(HostError):
    """Kart cevap verdi ama durum OK değil (ör. GET_IMU: E_STATE)."""

    def __init__(self, text, status):
        super().__init__(text)
        self.status = status


class TelemetryStats:
    def __init__(self):
        self.reset()

    def reset(self):
        self.received = 0
        self.gaps = 0              # kaybolan paket sayısı (SEQ boşlukları toplamı)
        self.dups = 0
        self.last_seq = None
        self.first_time = None
        self.last_time = None
        self.last = None

    def add(self, t: proto.Telem):
        if self.last_seq is not None:
            d = (t.seq - self.last_seq) & 0xFFFF
            if d == 0:
                self.dups += 1
            elif d < 0x8000:
                self.gaps += d - 1
        self.last_seq = t.seq
        self.received += 1
        self.first_time = self.first_time or t.rx_time
        self.last_time = t.rx_time
        self.last = t

    def rate_hz(self):
        if self.received < 2 or self.last_time == self.first_time:
            return 0.0
        return (self.received - 1) / (self.last_time - self.first_time)


class Host:
    def __init__(self, link, host_mac=eth.HOST_MAC, host_ip=eth.HOST_IP, board_ip=eth.BOARD_IP,
                 host_port=eth.HOST_PORT, fcs_flag=1, log=None):
        self.link = link
        self.host_mac = bytes(host_mac)
        self.host_ip = host_ip
        self.board_ip = board_ip
        self.host_port = host_port
        self.fcs_flag = fcs_flag            # host -> kart satır biçimi: None, 0 ya da 1
        self.log = log                      # fonksiyon(yön, metin)
        self.board_mac = None
        self.arp_mute = False               # T07: ARP cevabı kesilir
        self.epoch = None
        self.epoch_changes = 0
        self.ip_ident = itertools.count(1)
        # REQ_ID her süreçte rastgele bir yerden başlar. 1'den başlasaydı, 10 s içinde
        # art arda çalıştırılan iki süreç aynı REQ_ID'yi başka içerikle gönderir, kart da
        # ikincisini E_CONFLICT ile reddederdi (tasarım 16.4): START "gitmiş" görünür ama
        # ölçüm başlamazdı.
        self._req_ids = itertools.count(int.from_bytes(os.urandom(2), "big") or 1)
        self._seq = collections.defaultdict(int)       # kanal -> son SEQ
        self._cv = threading.Condition()
        self._pending = {}                  # req_id -> [Response|None]
        self._reports = collections.deque(maxlen=4000)
        self.last_report = {}               # tür -> son rapor
        self.telem = TelemetryStats()
        self.telem_log = collections.deque(maxlen=2000)
        self._ping_waiters = {}
        self.listeners = []                 # fonksiyon(olay, veri): 'report', 'telem', 'response', 'frame_err'
        # Arka plandaki GET_IMU poller'ları ve "kısa sessizlik" isteği (rapor 092): testler sayaç
        # görüntüsü alırken poller'lar o an istek göndermez.
        self._pollers = set()
        self._run_ok = threading.Event()
        self._run_ok.set()
        self._quiet_depth = 0
        self._quiet_lock = threading.Lock()
        self._quiet_exempt = set()          # quiesce sırasında yine de çalışan poller'lar
        self.c = collections.Counter()      # host sayaçları
        link.on_line = self._on_line

    # ------------------------------------------------------------ yardımcılar
    def _emit(self, kind, data):
        for fn in list(self.listeners):
            try:
                fn(kind, data)
            except Exception:
                pass

    def _log(self, direction, text):
        if self.log is not None:
            self.log(direction, text)

    def next_req_id(self):
        r = next(self._req_ids) & 0xFFFF
        if r == 0:
            r = next(self._req_ids) & 0xFFFF
        return r

    def next_seq(self, channel):
        self._seq[channel] = (self._seq[channel] + 1) & 0xFFFF
        return self._seq[channel]

    # ------------------------------------------------------------ alma
    def _on_line(self, text, rx_time):
        self.c["rx_lines"] += 1
        bl = lines.parse_board_line(text, rx_time)
        if bl.kind == "json":
            self.c["rx_json"] += 1
            self._log("rx", text)
            self._on_report(bl.report, rx_time)
        elif bl.kind == "frame":
            self.c["rx_frames"] += 1
            self._on_frame(bl.frame, rx_time, text)
        else:
            self.c["rx_bad_" + bl.error] += 1
            self._log("rx!", "%s: %s" % (bl.error, text[:120]))
            self._emit("frame_err", (bl.error, text))

    def _on_report(self, rep, rx_time):
        rep = dict(rep)
        rep["_rx"] = rx_time
        t = rep.get("type")
        if t in ("boot", "hello", "st", "state") and "ep" in rep:
            if self.epoch is not None and rep["ep"] != self.epoch:
                self.epoch_changes += 1
                self._seq.clear()           # kart yeniden başladı: SEQ'ler 0'dan
            self.epoch = rep["ep"]
        with self._cv:
            self._reports.append(rep)
            self.last_report[t] = rep
            self._cv.notify_all()
        self._emit("report", rep)

    def _on_frame(self, frame, rx_time, text):
        f = eth.parse_frame(frame, self.host_mac)
        if not f.ok:
            for e in f.errors:
                self.c["frame_err_" + e] += 1
            self._log("rx!", "frame %s" % ",".join(f.errors))
            self._emit("frame_err", (",".join(f.errors), text))
            return
        if f.kind == "ARP":
            self._on_arp(f)
        elif f.kind == "ICMP":
            if f.icmp_type == 0:
                self.c["icmp_reply"] += 1
                w = self._ping_waiters.get((f.icmp_ident, f.icmp_seq))
                if w is not None:
                    w[0] = rx_time
                    w[1].set()
        elif f.kind == "UDP":
            if f.ip_src != self.board_ip or f.ip_dst != self.host_ip:
                self.c["udp_wrong_addr"] += 1
                return
            if self.board_mac is None:
                self.board_mac = f.src
            self._on_udp(f, rx_time)
        else:
            self.c["frame_other"] += 1

    def _on_arp(self, f):
        if f.arp_sender_ip == self.board_ip:
            if self.board_mac != f.arp_sender_mac:
                self.board_mac = f.arp_sender_mac
                self._log("info", "kart MAC = %s" % eth.mac_str(self.board_mac))
            with self._cv:
                self._cv.notify_all()
        if f.arp_op == 1 and f.arp_target_ip == self.host_ip:
            self.c["arp_req_rx"] += 1
            if self.arp_mute:
                self.c["arp_req_ignored"] += 1
                self._log("info", "ARP isteği cevapsız bırakıldı (arp_mute)")
                return
            reply = eth.build_arp(2, self.host_mac, self.host_ip, f.arp_sender_mac, f.arp_sender_ip)
            self.c["arp_reply_tx"] += 1
            self.send_frame(reply, note="ARP reply")
        elif f.arp_op == 2:
            self.c["arp_reply_rx"] += 1

    def _on_udp(self, f, rx_time):
        try:
            pkt = proto.decode(f.udp_payload)
        except proto.PacketError as e:
            self.c["pkt_err_" + e.reason] += 1
            self._log("rx!", "paket %s (port %d)" % (e.reason, f.sport))
            return
        if f.sport == proto.PORT_CTRL:
            if f.dport != self.host_port:
                self.c["resp_wrong_port"] += 1
            if not pkt.is_response:
                self.c["resp_not_resp"] += 1
                return
            try:
                resp = proto.parse_response(pkt)
            except proto.PacketError as e:
                self.c["resp_" + e.reason] += 1
                return
            resp.rx_time = rx_time
            self.c["responses"] += 1
            if pkt.type != proto.CMD_GET_IMU:
                # GET_IMU 100 Hz gelebilir: günlüğü boğmasın, ImuPoller sayıyor (rapor 086)
                self._log("rx", "cevap %s REQ_ID %d %s lat %d µs" %
                          (proto.TYPE_NAMES.get(pkt.type, pkt.type), pkt.req_id, resp.status_name, resp.lat_us))
            with self._cv:
                slot = self._pending.get(pkt.req_id)
                if slot is not None and slot[0] is None:
                    slot[0] = resp
                    self._cv.notify_all()
                else:
                    self.c["resp_unmatched"] += 1
            if self.epoch is not None and resp.epoch != self.epoch:
                self.epoch_changes += 1
            self.epoch = resp.epoch
            self._emit("response", resp)
        elif f.sport == proto.PORT_TELEM:
            try:
                t = proto.parse_telem(pkt)
            except proto.PacketError:
                self.c["telem_bad"] += 1
                return
            t.rx_time = rx_time
            self.c["telem_rx"] += 1
            self.telem.add(t)
            self.telem_log.append(t)
            self._emit("telem", t)
        else:
            self.c["udp_other_port"] += 1

    # ------------------------------------------------------------ gönderme
    def send_line(self, data: bytes, mode="whole", seed=1234):
        self._log("tx", data.decode("ascii", "replace").rstrip("\n")[:200])
        self.link.write_fragmented(data, mode=mode, seed=seed)

    def send_frame(self, frame: bytes, fcs_flag="default", mode="whole", seed=1234, note="", quiet=False):
        flag = self.fcs_flag if fcs_flag == "default" else fcs_flag
        line = lines.frame_line(frame, flag)
        self.c["tx_frames"] += 1
        if not quiet:
            self._log("tx", (note + ": " if note else "") + line.decode().rstrip("\n")[:160])
        self.link.write_fragmented(line, mode=mode, seed=seed)

    def udp_frame(self, dport: int, payload: bytes, sport=None, bad_ip=False, bad_udp=False, dst_mac=None):
        if self.board_mac is None and dst_mac is None:
            self.resolve()
        return eth.build_udp_frame(dst_mac or self.board_mac, self.host_mac, self.host_ip, self.board_ip,
                                   sport or self.host_port, dport, payload, ident=next(self.ip_ident),
                                   bad_ip_checksum=bad_ip, bad_udp_checksum=bad_udp)

    def send_udp(self, dport: int, payload: bytes, **kw):
        mode = kw.pop("mode", "whole")
        flag = kw.pop("fcs_flag", "default")
        quiet = kw.pop("quiet", False)
        self.send_frame(self.udp_frame(dport, payload, **kw), fcs_flag=flag, mode=mode, quiet=quiet)

    # ------------------------------------------------------------ ARP / ping
    def resolve(self, timeout=1.0, retries=3):
        """Kartın MAC'ini ARP ile öğrenir (kartın MAC'i UID'den türetilir, sabit değil)."""
        for _ in range(retries):
            req = eth.build_arp(1, self.host_mac, self.host_ip, b"\x00" * 6, self.board_ip)
            self.send_frame(req, note="ARP request")
            deadline = time.perf_counter() + timeout
            with self._cv:
                while self.board_mac is None:
                    left = deadline - time.perf_counter()
                    if left <= 0:
                        break
                    self._cv.wait(left)
            if self.board_mac is not None:
                return self.board_mac
        raise HostError("kart ARP'ye cevap vermedi (%s)" % self.board_ip)

    def ping(self, timeout=1.0, seq=None):
        """ICMP echo; RTT ms ya da None."""
        if self.board_mac is None:
            self.resolve()
        ident = 0x4848
        seq = seq if seq is not None else next(self._req_ids) & 0xFFFF
        ev = threading.Event()
        slot = [None, ev]
        self._ping_waiters[(ident, seq)] = slot
        t0 = time.perf_counter()
        self.send_frame(eth.build_icmp_echo(self.board_mac, self.host_mac, self.host_ip, self.board_ip,
                                            ident, seq, ip_ident=next(self.ip_ident)), note="ping")
        ok = ev.wait(timeout)
        self._ping_waiters.pop((ident, seq), None)
        return (slot[0] - t0) * 1000.0 if ok else None

    # ------------------------------------------------------------ konsol
    def report_mark(self):
        with self._cv:
            return len(self._reports), time.perf_counter()

    def wait_report(self, types, since=None, timeout=1.0, pred=None):
        """since = report_mark() sonucu; o andan sonra gelen ilk uygun rapor."""
        types = (types,) if isinstance(types, str) else tuple(types)
        t_since = since[1] if since else time.perf_counter()
        deadline = time.perf_counter() + timeout
        with self._cv:
            while True:
                for rep in self._reports:
                    if rep["_rx"] >= t_since and rep.get("type") in types and (pred is None or pred(rep)):
                        if not rep.get("_taken"):
                            rep["_taken"] = True
                            return rep
                left = deadline - time.perf_counter()
                if left <= 0:
                    return None
                self._cv.wait(left)

    def reports_since(self, mark, types=None):
        with self._cv:
            return [r for r in self._reports if r["_rx"] >= mark[1] and (types is None or r.get("type") in types)]

    def console(self, cmd: str, expect=("ack", "error"), timeout=1.0, mode="whole"):
        mark = self.report_mark()
        self.send_line(lines.console_line(cmd), mode=mode)
        return self.wait_report(expect, mark, timeout)

    def hello(self, timeout=1.0):
        return self.console("HELLO", ("hello",), timeout)

    def state(self, timeout=1.0):
        return self.console("STATE", ("state",), timeout)

    def hold(self, ms: int):
        return self.console("HOLD %d" % ms)

    def txdelay(self, ms: int):
        return self.console("TXDELAY %d" % ms)

    # ------------------------------------------------------------ komutlar
    def command_packet(self, type_, payload=b"", req_id=None, seq=None):
        req_id = self.next_req_id() if req_id is None else req_id
        seq = self.next_seq("cmd") if seq is None else seq
        return req_id, proto.encode(proto.Packet(type_, 0, seq, req_id, payload))

    @contextlib.contextmanager
    def quiesce(self, timeout=1.0, exempt=None):
        """Arka plandaki GET_IMU poller'larını kısa süre bekletir (rapor 092). Sayaç eşitlikleri (tasarım
        18.1 kural 3) iki ucun aynı anı görmesini ister: hatta yolda satır varken bu olamaz. İzleme
        açık kalır; yalnız görüntü anında (drain + GET_STATS, ~0,5 s) istek gitmez.
        exempt: bu sürede çalışmaya devam eden poller (T08 kendi ölçümü sırasında diğer GET_IMU
        istemcisini bekletir: iki istemci aynı anda isterse cevaplardan biri diğerinin satırını da bekler)."""
        with self._quiet_lock:
            self._quiet_depth += 1
            self._run_ok.clear()
            if exempt is not None:
                self._quiet_exempt.add(exempt)
        deadline = time.perf_counter() + timeout
        for p in list(self._pollers):
            if p is not exempt:
                p.idle.wait(max(0.0, deadline - time.perf_counter()))
        try:
            yield
        finally:
            with self._quiet_lock:
                self._quiet_depth -= 1
                self._quiet_exempt.discard(exempt)
                if self._quiet_depth == 0:
                    self._run_ok.set()

    def command(self, type_, payload=b"", req_id=None, timeout=0.5, retries=2, mode="whole", raw=None, quiet=False,
                fcs_flag="default"):
        """Komutu gönderir, cevabı REQ_ID ile bekler. Zaman aşımında AYNI REQ_ID ile tekrar
        (kart tekrarı yan etkisiz cevaplar, tasarım 16.4). Cevap yoksa HostError."""
        req_id = self.next_req_id() if req_id is None else req_id
        with self._cv:
            self._pending[req_id] = [None]
        t_first = time.perf_counter()
        try:
            for attempt in range(1, retries + 2):
                if raw is not None:
                    data = raw
                else:
                    data = proto.encode(proto.Packet(type_, 0, self.next_seq("cmd"), req_id, payload))
                t0 = time.perf_counter()
                self.send_udp(proto.PORT_CTRL, data, mode=mode, quiet=quiet, fcs_flag=fcs_flag)
                self.c["cmd_tx"] += 1
                deadline = t0 + timeout
                with self._cv:
                    while self._pending[req_id][0] is None:
                        left = deadline - time.perf_counter()
                        if left <= 0:
                            break
                        self._cv.wait(left)
                    resp = self._pending[req_id][0]
                if resp is not None:
                    resp.rtt_ms = (resp.rx_time - t0) * 1000.0
                    resp.attempts = attempt
                    return resp
                self.c["cmd_timeout"] += 1
            raise HostError("komut %s REQ_ID %d: cevap yok (%d deneme, %.0f ms)" %
                            (proto.TYPE_NAMES.get(type_, type_), req_id, retries + 1,
                             (time.perf_counter() - t_first) * 1000))
        finally:
            with self._cv:
                self._pending.pop(req_id, None)

    def start(self, **kw):
        return self.command(proto.CMD_START, **kw)

    def stop(self, **kw):
        return self.command(proto.CMD_STOP, **kw)

    def set_rate(self, hz, **kw):
        return self.command(proto.CMD_SET_RATE, proto.payload_set_rate(hz), **kw)

    def get_stats_page(self, page, **kw):
        r = self.command(proto.CMD_GET_STATS, proto.payload_get_stats(page), **kw)
        if not r.ok:
            raise HostError("GET_STATS %d: %s" % (page, r.status_name))
        return proto.parse_stats_page(r.extra)

    def get_counters(self, tries=3):
        """Sayfa 0..2 -> {ad: değer}; ayrıca '_tick'. Rapor 092: kart görüntüyü sayfa 0'da alır, 1 ve 2
        aynı görüntüden ve aynı tick'le döner; tick'ler farklıysa (araya başka bir istemcinin sayfa 0'ı
        girdi) üç sayfa yeniden okunur."""
        for _ in range(tries):
            pages = [self.get_stats_page(page) for page in range(3)]
            tick = pages[0].tick
            if all(sp.tick == tick for sp in pages):
                break
        vals = []
        for sp in pages:
            vals.extend(sp.values)
        out = dict(zip(COUNTER_NAMES, vals))
        out["_tick"] = tick
        return out

    def get_env(self):
        return proto.env_channels(self.get_stats_page(3))

    def get_imu(self, timeout=0.05, retries=0):
        """GET_IMU (rapor 086): en son IMU örneği + roll / pitch / yaw. Yalnız START ile STOP
        arasında OK; değilse StatusError (E_STATE). Cevap gelmezse HostError."""
        r = self.command(proto.CMD_GET_IMU, timeout=timeout, retries=retries, quiet=True)
        if not r.ok:
            raise StatusError("GET_IMU: %s" % r.status_name, r.status)
        imu = proto.parse_imu_reply(r.extra)
        imu.lat_us = r.lat_us
        imu.req_id = r.req_id
        imu.rtt_ms = r.rtt_ms
        imu.rx_time = r.rx_time
        return imu

    # ------------------------------------------------------------ önemsiz veri
    def send_env(self, values, **kw):
        self.c["env_tx"] += 1
        p = proto.encode(proto.Packet(proto.ENV, 0, self.next_seq("env"), 0, proto.payload_env(values)))
        self.send_udp(proto.PORT_LOW, p, **kw)

    def send_bulk(self, seed, m, **kw):
        self.c["bulk_tx"] += 1
        p = proto.encode(proto.Packet(proto.BULK, 0, self.next_seq("bulk"), 0, proto.payload_bulk(seed, m)))
        self.send_udp(proto.PORT_LOW, p, **kw)

    def send_cal(self, offsets, **kw):
        self.c["cal_tx"] += 1
        p = proto.encode(proto.Packet(proto.CAL, 0, self.next_seq("cal"), 0, proto.payload_cal(offsets)))
        self.send_udp(proto.PORT_LOW, p, **kw)

    # ------------------------------------------------------------ hazırlık
    def connect(self, timeout=3.0):
        """HELLO + ARP. Kart açılışta NET_READY için ~1 s isteyebilir."""
        deadline = time.perf_counter() + timeout
        hello = None
        while hello is None and time.perf_counter() < deadline:
            hello = self.hello(timeout=0.7)
        if hello is None:
            raise HostError("HELLO cevabı yok: port / baud / kart açık mı?")
        self.resolve()
        self.wait_net_ready()
        return hello

    def wait_net_ready(self, timeout=3.0):
        """Kart açılışta IP task'ı başlatıp NET_READY'yi kurana kadar START E_STATE alır."""
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            st = self.state(timeout=0.7)
            if st and st.get("s", 0) & proto.STATE_NET_READY:
                return True
            time.sleep(0.2)
        return False

    def drain(self, quiet_s=0.3, timeout=3.0):
        """Hat sakinleşene kadar bekler (muhasebe görüntüsünden önce, tasarım 18.1 kural 3)."""
        deadline = time.perf_counter() + timeout
        last = self.link.rx_bytes
        quiet_since = time.perf_counter()
        while time.perf_counter() < deadline:
            time.sleep(0.05)
            if self.link.rx_bytes != last:
                last = self.link.rx_bytes
                quiet_since = time.perf_counter()
            elif time.perf_counter() - quiet_since >= quiet_s:
                return True
        return False


# ---------------------------------------------------------------- GET_IMU (rapor 086)
class ImuDeadlineStats:
    """GET_IMU deadline'ı KART ZAMANIYLA (rapor 091): her cevap, bir önceki GET_IMU cevabının kartta
    ölçülen M1 -> M6 süresini taşır (istek karta geldi -> cevabın son biti UART'tan çıktı).
    Windows / USB ve host'un kendi gecikmesi bu sayılara girmez.

    Ayrıca M1 -> M6 ikiye bölünür: işleme M1 -> M3 (o isteğin cevabındaki LAT_US) ve
    gönderim M3 -> M6 (TX queue'su + satırın hattaki süresi)."""

    def __init__(self, keep=2000):
        self.keep = keep
        self.reset()

    def reset(self):
        self.requests = 0
        self.ok = 0
        self.timeouts = 0
        self.errors = collections.Counter()      # durum adı -> sayı (E_STATE ...)
        self.board = collections.deque(maxlen=self.keep)    # M1 -> M6, ms
        self.proc = collections.deque(maxlen=self.keep)     # M1 -> M3, ms
        self.send = collections.deque(maxlen=self.keep)     # M3 -> M6, ms
        self.board_n = 0
        self.board_sum = 0.0
        self.board_max = 0.0
        self.le_target = 0
        self.le_deadline = 0
        self.over_deadline = 0
        self.same_index = 0                      # aynı örnek iki kez (istek 10 ms'den sık)
        self.index_gaps = 0                      # atlanan örnek (istek periyodu > 10 ms: beklenen)
        self.last_index = None
        self.last = None
        self._lat = collections.OrderedDict()    # REQ_ID -> o cevabın LAT_US'u (M1 -> M3)

    def add(self, imu):
        self.requests += 1
        self.ok += 1
        self._lat[imu.req_id] = imu.lat_us
        while len(self._lat) > 64:
            self._lat.popitem(last=False)
        lat = self._lat.pop(imu.prev_req_id, None) if imu.prev_req_id else None
        if lat is not None:                      # bu poller'ın bir önceki isteği: kart süresi geldi
            ms = imu.prev_us / 1000.0
            self.board.append(ms)
            self.proc.append(lat / 1000.0)
            self.send.append(max(0.0, ms - lat / 1000.0))
            self.board_n += 1
            self.board_sum += ms
            self.board_max = max(self.board_max, ms)
            if ms <= proto.IMU_TARGET_MS:
                self.le_target += 1
            if ms <= proto.IMU_DEADLINE_MS:
                self.le_deadline += 1
            else:
                self.over_deadline += 1
        if self.last_index is not None:
            d = (imu.index - self.last_index) & 0xFFFFFFFF
            if d == 0:
                self.same_index += 1
            elif d < 0x80000000:
                self.index_gaps += d - 1
        self.last_index = imu.index
        self.last = imu

    def add_timeout(self):
        self.requests += 1
        self.timeouts += 1

    def add_error(self, name):
        self.requests += 1
        self.errors[name] += 1

    def mean(self):
        return self.board_sum / self.board_n if self.board_n else 0.0

    @staticmethod
    def _pct(values, q):
        if not values:
            return 0.0
        v = sorted(values)
        return v[min(len(v) - 1, int(q * len(v)))]

    def percentile(self, q):
        return self._pct(self.board, q)

    def summary(self):
        return {
            "requests": self.requests, "ok": self.ok, "timeouts": self.timeouts, "errors": dict(self.errors),
            "board_n": self.board_n, "board_mean_ms": round(self.mean(), 3),
            "board_p99_ms": round(self.percentile(0.99), 3), "board_max_ms": round(self.board_max, 3),
            "le_10ms": self.le_target, "le_12ms": self.le_deadline, "over_12ms": self.over_deadline,
            "proc_max_ms": round(max(self.proc), 3) if self.proc else 0.0,
            "send_max_ms": round(max(self.send), 3) if self.send else 0.0,
            "same_index": self.same_index, "index_gaps": self.index_gaps,
        }


class ImuPoller:
    """GET_IMU'yu arka planda periyotla sorar (raporlar 086, 091; varsayılan 20 ms). Aynı anda tek
    istek: cevap gelmeden yenisi gönderilmez; cevap periyottan geç gelirse bir sonraki hemen gider."""

    def __init__(self, host, period_ms=proto.IMU_PERIOD_MS, timeout_ms=50.0, on_sample=None):
        self.host = host
        self.period_s = period_ms / 1000.0
        self.timeout_s = timeout_ms / 1000.0
        self.on_sample = on_sample
        self.stats = ImuDeadlineStats()
        self.last_error = None
        self._stop = threading.Event()
        self._thread = None
        self.idle = threading.Event()           # istek arasında (quiesce bunu bekler)
        self.idle.set()

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self):
        if self.running:
            return
        self._stop.clear()
        self.host._pollers.add(self)
        self._thread = threading.Thread(target=self._run, name="imu-poll", daemon=True)
        self._thread.start()

    def stop(self, wait=1.0):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(wait)
        self._thread = None
        self.host._pollers.discard(self)
        self.idle.set()

    def _run(self):
        next_t = time.perf_counter()
        while not self._stop.is_set():
            # Karar ve idle.clear() aynı kilit altında (rapor 092): quiesce _run_ok'u bu kilitle temizleyip
            # sonra idle'ı bekler. Kilit yoksa "izin var" kontrolü ile idle.clear() arasına quiesce girer,
            # idle'ı hâlâ set görür ve sayaç görüntüsü sırasında bir GET_IMU kaçar (kartta T06: L4 / L5 ±1).
            with self.host._quiet_lock:
                allowed = self.host._run_ok.is_set() or self in self.host._quiet_exempt
                if allowed:
                    self.idle.clear()
                else:
                    self.idle.set()
            if not allowed:                                                              # quiesce: bekle
                self.host._run_ok.wait(0.05)
                next_t = time.perf_counter()
                continue
            try:
                imu = self.host.get_imu(timeout=self.timeout_s)
                self.stats.add(imu)
                self.last_error = None
                if self.on_sample is not None:
                    self.on_sample(imu)
            except StatusError as e:
                self.stats.add_error(proto.STATUS_NAMES.get(e.status, str(e.status)))
                self.last_error = str(e)
                # Ölçüm yok (E_STATE): hattı boşuna doldurma, biraz bekle (rapor 092)
                self.idle.set()
                self._stop.wait(0.1)
            except HostError as e:
                self.stats.add_timeout()
                self.last_error = str(e)
            except Exception as e:          # bağlantı kapandı vb.
                self.last_error = repr(e)
                self._stop.wait(0.2)
            self.idle.set()
            next_t += self.period_s
            left = next_t - time.perf_counter()
            if left > 0:
                self._stop.wait(left)
            else:
                next_t = time.perf_counter()
