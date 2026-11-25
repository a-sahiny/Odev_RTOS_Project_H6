"""MP257'den UDP telemetri girişi (M8): F746 -> M33 -> RPMsg -> Linux telem_udp -> UDP 5002 -> PC.

Paket, M33'ün kurduğu 35 baytlık TELEM'dir; Host'un kendi decode'u ile çözülür, SEQ boşluğu
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
CMD_RATE = 36000            # bayt/s: USART6 çerçeveli hattın hızı 39,7 KB/s; seri portun yaptığı gibi yazan bekler
CMD_BURST = 2048            # frensiz yollanan bayt; RPMsg'nin 8 tamponuna ve M33'ün 8 KB sırasına sığar
CMD_MAX = 480               # bir komut datagramının bayt sınırı (telem_udp CMD_MAX)
LINE_HEADER = 4             # satır datagramı: 'L', bayrak (1 ilk, 2 son), sıra (2 bayt), veri
M33_RX_US = 563             # T2 - T1: 27 baytlık çerçevenin alınması; kartta ölçüldü (ort 562,7 µs, jitter 1,1)


class Span:
    """Bir sürenin canlı özeti, µs: en az, ortalama, en çok; jitter = en çok - en az."""

    def __init__(self):
        self.n = self.total = 0
        self.min = self.max = self.last = None

    def add(self, us):
        self.n += 1
        self.total += us
        self.last = us
        self.min = us if self.min is None else min(self.min, us)
        self.max = us if self.max is None else max(self.max, us)

    def cells(self):
        """Tablo hücreleri: en az, ort, en çok, jitter, son, adet."""
        if not self.n:
            return ("—",) * 5 + ("0",)
        return ("%d" % self.min, "%.1f" % (self.total / self.n), "%d" % self.max, "%d" % (self.max - self.min),
                "%d" % self.last, "%d" % self.n)


class UdpTelem:
    """UDP portunu dinler; her geçerli TELEM'i sayar ve on_telem'e verir. Port kullanımdaysa OSError."""

    def __init__(self, port=PORT, on_telem=None):
        self.telem = TelemetryStats()
        self.reset()
        self.on_telem = on_telem
        self.closed = False
        self.paused = False                 # T06: host okumayı bırakır
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.2)           # close() en geç bu kadar sonra okumayı bitirir
        threading.Thread(target=self._reader, name="udp-rx", daemon=True).start()

    def reset(self):
        self.telem.reset()
        self.bad = 0                        # çözülemeyen paket (CRC, uzunluk, tür)
        self.f746 = Span()                  # DRDY -> USART6 TX bitti (paketteki prev_us)
        self.m33 = Span()                   # ilk bayt T1 -> Linux'a verildi T5 (paketteki m33_us)
        self.tail = Span()                  # son bayt T2 -> Linux'a verildi T5
        self.whole = Span()                 # DRDY -> Linux'a verildi T5
        self.le_target = self.over_deadline = 0

    def _latency(self, t, prev):
        """prev_us ve m33_us bir önceki örneğin süreleridir; toplam için o örnek kayıpsız gelmiş olmalı."""
        if t.prev_valid:
            self.f746.add(t.prev_us)
        if not t.m33_us:
            return
        self.m33.add(t.m33_us)
        tail = t.m33_us - M33_RX_US
        self.tail.add(tail)
        if t.prev_valid and t.prev_seq == prev == (t.seq - 1) & 0xFFFF:
            whole = t.prev_us + tail
            self.whole.add(whole)
            self.le_target += whole <= proto.TELEM_TARGET_MS * 1000
            self.over_deadline += whole > proto.TELEM_DEADLINE_MS * 1000

    def _reader(self):
        while not self.closed:
            if self.paused:
                time.sleep(0.02)        # okumuyoruz: datagramlar işletim sisteminde birikir
                continue
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
            prev = self.telem.last_seq
            self.telem.add(t)
            self._latency(t, prev)
            if self.on_telem is not None:
                self.on_telem(t)
        self.sock.close()

    def close(self):
        self.closed = True


class LineJoin:
    """telem_udp'nin satır datagramlarını (en çok 2 parça) satıra çevirir; eksik parçalı satır atılır, sayılır."""

    def __init__(self):
        self.next = None
        self.buf = None                     # ilk parçası gelmiş satır
        self.lost = 0                       # gelmeyen datagram
        self.bad = 0                        # satır datagramı olmayan

    def add(self, data):
        if len(data) <= LINE_HEADER or data[:1] != b"L":
            self.bad += 1
            return None
        flags, seq = data[1], int.from_bytes(data[2:4], "big")
        if self.next is not None and seq != self.next:
            self.lost += (seq - self.next) & 0xFFFF
            self.buf = None
        self.next = (seq + 1) & 0xFFFF
        if flags & 1:
            self.buf = b""
        if self.buf is None:
            return None
        self.buf += data[LINE_HEADER:]
        if not flags & 2:
            return None
        line, self.buf = self.buf, None
        return line


class UdpLines:
    """F746'nın Linux üzerinden gelen satırları (UDP PORT + 1); her tam satır on_line'a verilir. Komutlar ters yönde."""

    def __init__(self, port=PORT + 1, on_line=None, board=BOARD_IP):
        self.join = LineJoin()
        self.target = (board, port)
        self._free_at = 0.0                 # yollananların hattan çıkacağı an
        self._send_lock = threading.Lock()
        self.on_line = on_line
        self.received = 0
        self.closed = False
        self.paused = False                 # T06: host okumayı bırakır
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.2)
        threading.Thread(target=self._reader, name="udp-lines", daemon=True).start()

    def _reader(self):
        while not self.closed:
            if self.paused:
                time.sleep(0.02)        # okumuyoruz: datagramlar işletim sisteminde birikir
                continue
            try:
                data = self.sock.recv(2048)
            except socket.timeout:
                continue
            except ConnectionResetError:
                continue            # kartta telem_udp yok: Windows sendto'nun hatasını burada verir
            line = self.join.add(data)
            if line is not None:
                self.received += 1
                if self.on_line is not None:
                    self.on_line(line)
        self.sock.close()

    def send(self, data):
        """Baytları karttaki telem_udp'ye yollar (o da M33 üzerinden F746'ya)."""
        with self._send_lock:
            for i in range(0, len(data), CMD_MAX):
                part = data[i:i + CMD_MAX]
                wait = self._free_at - time.perf_counter() - CMD_BURST / CMD_RATE
                if wait > 0:
                    time.sleep(wait)
                self.sock.sendto(part, self.target)
                self._free_at = max(self._free_at, time.perf_counter()) + len(part) / CMD_RATE

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
