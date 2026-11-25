"""MP257'den UDP telemetri girişi (M8): F746 -> M33 -> RPMsg -> Linux telem_udp -> UDP 5002 -> PC.

Paket, M33'ün kurduğu 33 baytlık TELEM'dir; Host'un kendi decode'u ile çözülür, SEQ boşluğu
TelemetryStats ile sayılır. Karttaki telem_udp ssh ile çalıştırılır.
"""

import socket
import subprocess
import threading
import time

from . import proto
from .hoststack import TelemetryStats

BOARD_IP = "192.168.7.1"
PORT = proto.PORT_TELEM
PROGRAM = "/root/telem_udp"


class Span:
    """Bir sürenin canlı özeti, µs: en az, ortalama, en çok; jitter = en çok - en az."""

    def __init__(self):
        self.n = self.total = 0
        self.min = self.max = None

    def add(self, us):
        self.n += 1
        self.total += us
        self.min = us if self.min is None else min(self.min, us)
        self.max = us if self.max is None else max(self.max, us)

    def row(self, name):
        if not self.n:
            return "%-5s %6s" % (name, "—")
        return "%-5s %6d %6.1f %6d %6d %6d" % (name, self.min, self.total / self.n, self.max, self.max - self.min, self.n)


class UdpTelem:
    """UDP portunu dinler; her geçerli TELEM'i sayar ve on_telem'e verir. Port kullanımdaysa OSError."""

    def __init__(self, port=PORT, on_telem=None):
        self.telem = TelemetryStats()
        self.bad = 0                        # çözülemeyen paket (CRC, uzunluk, tür)
        self.f746 = Span()                  # F746: DRDY -> USART6 son bit (paketteki prev_us)
        self.m33 = Span()                   # MP257: ilk bayt kesmesi T1 -> Linux'a verildi T5 (paketteki m33_us)
        self.on_telem = on_telem
        self.closed = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.2)           # close() en geç bu kadar sonra okumayı bitirir
        threading.Thread(target=self._reader, name="udp-rx", daemon=True).start()

    def _reader(self):
        while not self.closed:
            try:
                data = self.sock.recv(2048)
            except socket.timeout:
                continue
            try:
                t = proto.parse_telem(proto.decode(data))
            except proto.PacketError:
                self.bad += 1
                continue
            t.rx_time = time.perf_counter()
            self.telem.add(t)
            if t.prev_valid:
                self.f746.add(t.prev_us)
            if t.m33_us:
                self.m33.add(t.m33_us)
            if self.on_telem is not None:
                self.on_telem(t)
        self.sock.close()

    def close(self):
        self.closed = True


def pc_ip(board=BOARD_IP):
    """PC'nin karta giden hattaki adresi (USB ağı: 192.168.7.66). Paket yollanmaz."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect((board, PORT))
    ip = s.getsockname()[0]
    s.close()
    return ip


def run_board(pc, port=PORT, board=BOARD_IP):
    """Kartta telem_udp'yi başlatır. -tt: ssh kapatılınca kartta SIGHUP gelir, telem_udp M33'e '0'
    yollayıp düzgün kapanır. Hata çıktısı stdout'tan okunur."""
    return subprocess.Popen(["ssh", "-tt", "root@" + board, PROGRAM, pc, str(port)],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))   # exe'de konsol açılmasın
