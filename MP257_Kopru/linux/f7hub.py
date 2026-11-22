#!/usr/bin/env python3
"""f7hub: MP257 Linux'taki uygulama servisi. F746'nın tek istemcisi, M33'ün köprüsü, PC'nin sunucusu.

    tap-f7 üzerinden normal UDP soketi (192.168.50.1:6000 <-> 192.168.50.2:5000/5002)
        komut gönderir (REQ_ID eşleme + aynı REQ_ID ile tekrar), telemetriyi alır: her IMU örneği bir
        TELEM (ivme, jiroskop, roll / pitch / yaw ve bir önceki TELEM'in kart içi DRDY -> TX bitti süresi)
    UDP 127.0.0.1:7101 <- f7linkd'nin aktardığı F746 JSON raporları ve hat sayaçları
    /dev/ttyRPMSG0 <-> M33 (FreeRTOS ya da Zephyr; aynı satır protokolü: protocol/A35_M33_Protokolu.md)
    HTTP :8080 -> PC: panel (web/index.html), canlı olaylar (/events, SSE), komut (/api/cmd), durum (/api/status)
    CSV kaydı: her TELEM bir satır, --log-dir altına
Yalnız Python standart kütüphanesi.
"""

import argparse
import collections
import csv
import itertools
import json
import os
import queue
import socket
import struct
import sys
import termios
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from f7common import proto  # noqa: E402

CMD_TIMEOUT_S = 0.15        # tek deneme; F746 cevabı tipik < 15 ms + 1 Mbaud hat
CMD_ATTEMPTS = 3            # aynı REQ_ID ile: F746 tekrarı tanır (dedup penceresi 10 s)
HB_PERIOD_S = 0.1           # M33 kalp atışı (M33 500 ms'de A35_LOST der)


def log(msg):
    print(time.strftime("%H:%M:%S ") + msg, flush=True)


class Broker:
    """SSE yayını: her istemcinin kendi kuyruğu; yavaş istemci olay kaybeder, kimseyi durdurmaz."""

    def __init__(self):
        self.lock = threading.Lock()
        self.clients = set()

    def subscribe(self):
        q = queue.Queue(maxsize=500)
        with self.lock:
            self.clients.add(q)
        return q

    def unsubscribe(self, q):
        with self.lock:
            self.clients.discard(q)

    def publish(self, kind, data):
        msg = "event: %s\ndata: %s\n\n" % (kind, json.dumps(data, separators=(",", ":")))
        with self.lock:
            clients = list(self.clients)
        for q in clients:
            try:
                q.put_nowait(msg)
            except queue.Full:
                pass


class Hub:
    def __init__(self, args):
        self.args = args
        self.broker = Broker()
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", proto.HOST_PORT))
        self.json_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.json_sock.bind(("127.0.0.1", args.json_port))
        # REQ_ID rastgele başlar: art arda başlatılan iki hub aynı REQ_ID'yi başka içerikle yollamasın
        # (F746 bunu E_CONFLICT ile reddeder; eski host'taki aynı gerekçe).
        self.req_ids = itertools.count(int.from_bytes(os.urandom(2), "big") or 1)
        self.seq = collections.defaultdict(int)
        self.pending = {}                           # req_id -> [Event, cevap]
        self.state = {"f7_state": None, "f7_why": 0, "epoch": None, "meas": False, "rate_hz": None,
                      "telem": None, "m33": None, "m33_rtos": None, "linkd": None,
                      "hello": None, "telem_count": 0, "telem_gaps": 0, "last_telem_seq": None,
                      # kart içi gecikme (DRDY -> TX bitti), TELEM'lerin taşıdığı ölçümler
                      "lat_n": 0, "lat_last_ms": None, "lat_max_ms": 0.0, "lat_over12": 0}
        self.last_prev_seq = None
        self.m33_fd = None
        self.m33_lock = threading.Lock()
        self.csv = self.open_csv(args.log_dir)

    # ------------------------------------------------------------ yardımcılar
    def open_csv(self, log_dir):
        if not log_dir:
            return None
        os.makedirs(log_dir, exist_ok=True)
        path = os.path.join(log_dir, time.strftime("f7_%Y%m%d_%H%M%S.csv"))
        f = open(path, "w", newline="")
        w = csv.writer(f)
        w.writerow(["t_unix", "seq", "ax", "ay", "az", "gx", "gy", "gz", "roll_cdeg", "pitch_cdeg", "yaw_cdeg",
                    "flags", "prev_seq", "prev_us"])
        log("kayit: %s" % path)
        return (f, w)

    def write_csv(self, row):
        if self.csv is None:
            return
        f, w = self.csv
        with self.lock:
            w.writerow(row)
            f.flush()

    def next_req_id(self):
        r = next(self.req_ids) & 0xFFFF
        return r if r else (next(self.req_ids) & 0xFFFF)

    def next_seq(self, port):
        self.seq[port] = (self.seq[port] + 1) & 0xFFFF
        return self.seq[port]

    def set_f7_state(self, state, why=None):
        with self.lock:
            changed = state != self.state["f7_state"] or (why is not None and why != self.state["f7_why"])
            self.state["f7_state"] = state
            if why is not None:
                self.state["f7_why"] = why
            self.state["meas"] = bool(state & proto.STATE_MEAS_EN)
            why_now = self.state["f7_why"]
        if changed:
            self.m33_send("ST %d %d" % (state, why_now))
            self.broker.publish("f7state", {"state": state, "text": proto.state_text(state), "why": why_now})

    # ------------------------------------------------------------ komutlar
    def command(self, type_, payload=b""):
        """Komutu 5000'e yollar, aynı REQ_ID ile en çok 3 kez dener. Döner: cevap sözlüğü ya da None."""
        req_id = self.next_req_id()
        pkt = proto.encode(type_, self.next_seq(proto.PORT_CTRL), req_id, payload)
        slot = [threading.Event(), None]
        with self.lock:
            self.pending[req_id] = slot
        try:
            t0 = time.monotonic()
            for attempt in range(1, CMD_ATTEMPTS + 1):
                self.sock.sendto(pkt, (proto.BOARD_IP, proto.PORT_CTRL))
                if slot[0].wait(CMD_TIMEOUT_S):
                    resp = slot[1]
                    resp["attempts"] = attempt
                    resp["rtt_ms"] = round((time.monotonic() - t0) * 1000.0, 2)
                    return resp
            return None
        finally:
            with self.lock:
                self.pending.pop(req_id, None)

    def do_cmd(self, req):
        """PC'den ya da M33 butonundan gelen istek. Döner: JSON'a yazılabilir sonuç."""
        name = req.get("cmd")
        if name == "start":
            resp = self.command(proto.CMD_START)
        elif name == "stop":
            resp = self.command(proto.CMD_STOP)
        elif name == "rate":
            hz = int(req.get("hz", proto.RATE_DEFAULT_HZ))
            if hz not in proto.RATES_HZ:
                return {"ok": False, "error": "hz: 10, 20, 50, 100 ya da 200"}
            resp = self.command(proto.CMD_SET_RATE, struct.pack(">H", hz))
            if resp and resp["status"] == 0:
                with self.lock:
                    self.state["rate_hz"] = hz
        elif name == "stats":
            page = int(req.get("page", 0))
            if not 0 <= page < proto.STATS_PAGES:
                return {"ok": False, "error": "page 0..3"}
            resp = self.command(proto.CMD_GET_STATS, bytes([page]))
            if resp and resp["status"] == 0:
                resp["stats"] = proto.parse_stats_page(resp["extra"])
        elif name == "limit":
            cdeg = int(req.get("cdeg", 3000))
            if not 100 <= cdeg <= 18000:
                return {"ok": False, "error": "cdeg 100..18000"}
            self.args.limit_cdeg = cdeg
            self.m33_send("LIM %d" % cdeg)
            return {"ok": True}
        elif name == "console":
            line = str(req.get("line", "")).encode("ascii", "replace")
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.5)
            try:
                s.sendto(line, ("127.0.0.1", self.args.ctl_port))
                reply = s.recv(16).decode()
            except OSError:
                reply = "f7linkd yok"
            finally:
                s.close()
            return {"ok": reply == "ok", "reply": reply}
        else:
            return {"ok": False, "error": "bilinmeyen komut"}

        if resp is None:
            return {"ok": False, "error": "cevap yok (3 deneme)"}
        out = {k: v for k, v in resp.items() if k != "extra"}
        out["ok"] = resp["status"] == 0
        out["state_text"] = proto.state_text(resp["state"])
        return out

    # ------------------------------------------------------------ alıcılar
    def udp_rx(self):
        """192.168.50.2'den gelen her UDP: komut cevabı (5000) ya da telemetri (5002)."""
        while True:
            data, addr = self.sock.recvfrom(512)
            if addr[0] != proto.BOARD_IP:
                continue
            try:
                type_, flags, seq, req_id, payload = proto.decode(data)
            except proto.PacketError:
                continue
            if flags & proto.FLAG_RESP:
                try:
                    resp = proto.parse_response(payload)
                except proto.PacketError:
                    continue
                resp.update(type=proto.TYPE_NAMES.get(type_, type_), req_id=req_id)
                self.set_f7_state(resp["state"])
                with self.lock:
                    slot = self.pending.get(req_id)
                if slot is not None and not slot[0].is_set():
                    slot[1] = resp
                    slot[0].set()
            elif type_ == proto.TELEM:
                self.on_telem(seq, payload)

    def on_telem(self, seq, payload):
        """Her IMU örneği: SEQ boşluğu, kart içi gecikme (bir önceki TELEM'in), M33'e TM + AT, PC'ye SSE, CSV."""
        try:
            t = proto.parse_telem(payload)
        except proto.PacketError:
            return
        t["seq"] = seq
        with self.lock:
            last = self.state["last_telem_seq"]
            if last is not None:
                d = (seq - last) & 0xFFFF
                if 1 < d < 0x8000:
                    self.state["telem_gaps"] += d - 1
            self.state["last_telem_seq"] = seq
            self.state["telem_count"] += 1
            self.state["telem"] = t
            # Aynı ölçüm iki TELEM'de gelebilir (önceki henüz hattayken bir sonraki kurulduysa): bir kez say.
            if t["prev_seq"] is not None and t["prev_seq"] != self.last_prev_seq:
                self.last_prev_seq = t["prev_seq"]
                ms = t["prev_us"] / 1000.0
                self.state["lat_n"] += 1
                self.state["lat_last_ms"] = ms
                self.state["lat_max_ms"] = max(self.state["lat_max_ms"], ms)
                if ms > proto.TELEM_DEADLINE_MS:
                    self.state["lat_over12"] += 1
        # İki satır tek yazımda: 200 Hz'de RPMsg mesaj sayısı yarıya iner (M33 satırları ayırır).
        self.m33_send("TM %d %d %d %d %d %d %d\nAT %d %d %d %d" % (
            seq, *t["acc"], *t["gyro"], t["roll_cdeg"], t["pitch_cdeg"], t["yaw_cdeg"], t["flags"]))
        self.broker.publish("telem", t)
        self.write_csv([round(time.time(), 4), seq, *t["acc"], *t["gyro"], t["roll_cdeg"], t["pitch_cdeg"],
                        t["yaw_cdeg"], t["flags"], "" if t["prev_seq"] is None else t["prev_seq"],
                        "" if t["prev_us"] is None else t["prev_us"]])

    def json_rx(self):
        """f7linkd'den: F746 JSON raporları ve {"type":"linkd"} hat sayaçları."""
        while True:
            data, _ = self.json_sock.recvfrom(2048)
            try:
                rep = json.loads(data)
            except ValueError:
                continue
            t = rep.get("type")
            if t == "linkd":
                with self.lock:
                    self.state["linkd"] = rep
                self.broker.publish("linkd", rep)
                continue
            if t == "hello":
                with self.lock:
                    self.state["hello"] = rep
            if t in ("boot", "hello", "st", "state") and "ep" in rep:
                with self.lock:
                    if self.state["epoch"] is not None and rep["ep"] != self.state["epoch"]:
                        self.seq.clear()            # F746 yeniden başladı: SEQ'ler sıfırdan
                        self.state["last_telem_seq"] = None
                    self.state["epoch"] = rep["ep"]
            if "s" in rep:
                self.set_f7_state(int(rep["s"]), int(rep.get("why", 0)))
            elif t == "fault":
                st = self.state["f7_state"] or 0
                st = (st | proto.STATE_FAULT) if rep.get("set") else (st & ~proto.STATE_FAULT)
                self.set_f7_state(st, int(rep.get("why", 0)))
            self.broker.publish("report", rep)

    # ------------------------------------------------------------ M33
    def m33_send(self, text):
        with self.m33_lock:
            fd = self.m33_fd
            if fd is None:
                return
            try:
                os.write(fd, (text + "\n").encode())
            except OSError:
                self.m33_fd = None                  # M33 durduruldu: m33_thread yeniden açar
                try:
                    os.close(fd)
                except OSError:
                    pass

    def m33_thread(self):
        """ttyRPMSG0'ı açar (yoksa 2 s'de bir dener), M33 satırlarını okur."""
        while True:
            try:
                fd = os.open(self.args.rpmsg, os.O_RDWR | os.O_NOCTTY)
            except OSError:
                time.sleep(2.0)
                continue
            attr = termios.tcgetattr(fd)
            attr[0] = 0
            attr[1] = 0                             # -onlcr: \n'yi \r\n'ye çevirme
            attr[3] = 0                             # -echo -icanon
            attr[6][termios.VMIN] = 1
            attr[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attr)
            with self.m33_lock:
                self.m33_fd = fd
            log("M33 kanali acik: %s" % self.args.rpmsg)
            self.m33_send("LIM %d" % self.args.limit_cdeg)
            buf = bytearray()
            while True:
                try:
                    data = os.read(fd, 256)
                except OSError:
                    data = b""
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    i = buf.index(b"\n")
                    line = bytes(buf[:i]).decode("ascii", "replace").strip()
                    del buf[:i + 1]
                    self.on_m33_line(line)
            log("M33 kanali kapandi")
            with self.m33_lock:
                if self.m33_fd == fd:
                    self.m33_fd = None
            try:
                os.close(fd)
            except OSError:
                pass

    def on_m33_line(self, line):
        parts = line.split()
        if not parts:
            return
        if parts[0] == "SUP" and len(parts) == 9:
            names = ("mode", "tm_age_ms", "tm_count", "tm_gaps", "dt_min_us", "dt_max_us", "hb_age_ms", "bad")
            sup = dict(zip(names, (int(x) for x in parts[1:])))
            sup["mode_text"] = ("WAIT_A35", "NO_DATA", "RUN", "ALARM", "F7_FAULT", "A35_LOST")[sup["mode"]] \
                if sup["mode"] < 6 else "?"
            with self.lock:
                self.state["m33"] = sup
            self.broker.publish("m33", sup)
        elif parts[0] == "BTN":
            # USER2: ölçüm açıksa durdur, kapalıysa başlat
            cmd = "stop" if self.state["meas"] else "start"
            threading.Thread(target=lambda: self.broker.publish("btn", {"cmd": cmd, "result": self.do_cmd({"cmd": cmd})}),
                             daemon=True).start()
        elif parts[0] == "RDY":
            with self.lock:
                self.state["m33_rtos"] = " ".join(parts[1:])
            self.m33_send("LIM %d" % self.args.limit_cdeg)
            self.broker.publish("m33_rdy", {"rtos": " ".join(parts[1:])})

    def heartbeat(self):
        for n in itertools.count():
            self.m33_send("HB %d" % (n & 0xFFFF))
            time.sleep(HB_PERIOD_S)

    # ------------------------------------------------------------ başlat
    def start(self):
        for target in (self.udp_rx, self.json_rx, self.m33_thread, self.heartbeat):
            threading.Thread(target=target, daemon=True, name=target.__name__).start()
        hub = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *a):
                pass

            def send_json(self, obj, code=200):
                body = json.dumps(obj, separators=(",", ":")).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    with open(os.path.join(HERE, "web", "index.html"), "rb") as f:
                        body = f.read()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == "/api/status":
                    with hub.lock:
                        snap = dict(hub.state)
                    snap["limit_cdeg"] = hub.args.limit_cdeg
                    self.send_json(snap)
                elif self.path == "/events":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/event-stream")
                    self.send_header("Cache-Control", "no-cache")
                    self.end_headers()
                    q = hub.broker.subscribe()
                    try:
                        while True:
                            try:
                                msg = q.get(timeout=15)
                            except queue.Empty:
                                msg = ": canli\n\n"     # bağlantı açık kalsın
                            self.wfile.write(msg.encode())
                            self.wfile.flush()
                    except OSError:
                        pass
                    finally:
                        hub.broker.unsubscribe(q)
                else:
                    self.send_json({"error": "yok"}, 404)

            def do_POST(self):
                if self.path != "/api/cmd":
                    self.send_json({"error": "yok"}, 404)
                    return
                n = int(self.headers.get("Content-Length", "0"))
                try:
                    req = json.loads(self.rfile.read(min(n, 1024)) or b"{}")
                    self.send_json(hub.do_cmd(req))
                except (ValueError, TypeError) as e:
                    self.send_json({"ok": False, "error": str(e)}, 400)

        srv = ThreadingHTTPServer(("0.0.0.0", self.args.http_port), Handler)
        srv.daemon_threads = True
        log("panel: http://<kart-ip>:%d/" % self.args.http_port)
        srv.serve_forever()


def main():
    ap = argparse.ArgumentParser(description="F746 istemcisi + M33 koprusu + PC paneli")
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--json-port", type=int, default=7101, help="f7linkd'nin rapor yolladigi port")
    ap.add_argument("--ctl-port", type=int, default=7100, help="f7linkd konsol komut portu")
    ap.add_argument("--rpmsg", default="/dev/ttyRPMSG0", help="M33 kanali")
    ap.add_argument("--limit-cdeg", type=int, default=3000, help="M33 egim alarm siniri (0,01 derece)")
    ap.add_argument("--log-dir", default="/home/root/f7_kayit", help="CSV klasoru; bos = kayit yok")
    Hub(ap.parse_args()).start()


if __name__ == "__main__":
    main()
