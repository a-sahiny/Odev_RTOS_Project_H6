"""Hat katmanı: seri port ya da simülatör. Satırları ayırır, yazmayı parçalayabilir (T01).

Akış kontrolü yok (tasarım 13.3): host okumayı bırakırsa (pause_reading) baytlar
işletim sistemi / USB tarafında birikir ya da kaybolur; kart bloklanmaz.
"""

import random
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:          # pyserial yoksa yalnız simülatör çalışır
    serial = None


def list_ports():
    """[(aygıt, açıklama)]; ST-LINK VCP genelde 'STMicroelectronics STLink Virtual COM Port'."""
    if serial is None:
        return []
    return [(p.device, p.description) for p in serial.tools.list_ports.comports()]


STLINK_V3_PIDS = (0x374D, 0x374E, 0x374F, 0x3753, 0x3754)     # MP257'nin ST-LINK'i; F746'nınki V2-1


def guess_stlink_port():
    """F746'nın ST-LINK portu; MP257'nin V3 ST-LINK portları seçilmez. Yoksa None."""
    if serial is None:
        return None
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x0483 and p.pid not in STLINK_V3_PIDS:
            return p.device
    return None


class Link:
    """Ortak taraf: satır ayırma, sayaçlar, parçalı yazma."""

    def __init__(self):
        self.on_line = None            # fonksiyon(text, rx_time)
        self.on_raw_tx = None          # fonksiyon(bytes) — günlük için
        self.rx_bytes = 0
        self.rx_lines = 0
        self.tx_bytes = 0
        self.tx_lines = 0
        self._rx_buf = bytearray()
        # Yeniden girilebilir: bir test hattı hold() ile ayırırken kendi write()'ları da geçer.
        self._tx_lock = threading.RLock()
        self._reading = threading.Event()
        self._reading.set()
        self._line_lock = threading.Lock()     # seri okuma ve Linux (UDP) thread'leri satırı sırayla verir
        self.linux_lines = 0                   # F746'nın Linux üzerinden gelen satırı
        self.linux_tx = 0                      # Linux üzerinden yollanan bayt
        self.route = None                      # fonksiyon(bytes) -> bool: Linux yolu aldıysa True
        self.linux_active = None               # fonksiyon() -> bool: Linux okuyor
        self.on_pause = None                   # fonksiyon(bool): Linux (UDP) alımını da durdur
        self.closed = False

    # ---- alma
    def _feed(self, data: bytes):
        now = time.perf_counter()
        self.rx_bytes += len(data)
        self._rx_buf += data
        while True:
            i = self._rx_buf.find(b"\n")
            if i < 0:
                break
            line = bytes(self._rx_buf[:i]).decode("ascii", errors="replace")
            del self._rx_buf[:i + 1]
            self._deliver(line, now)
        if len(self._rx_buf) > 4096:           # satırsız çöp: biriktirme
            self._rx_buf.clear()

    def _deliver(self, line, now):
        with self._line_lock:
            self.rx_lines += 1
            if self.on_line is not None:
                self.on_line(line, now)

    def feed_line(self, data: bytes):
        """Linux üzerinden (UDP) gelen bir tam satır, '\\n' dahil."""
        if not self.closed:
            self.rx_bytes += len(data)
            self.linux_lines += 1
            self._deliver(data.rstrip(b"\n").decode("ascii", errors="replace"), time.perf_counter())

    @property
    def via_linux(self):
        """Komut ve cevaplar şu an Linux üzerinden mi?"""
        return self.linux_active is not None and self.linux_active()

    def pause_reading(self, paused: bool):
        """T06: host okumayı bırakır / yeniden başlar."""
        if self.on_pause is not None:
            self.on_pause(paused)
        if paused:
            self._reading.clear()
        else:
            self._reading.set()

    # ---- gönderme
    def _write(self, data: bytes):
        raise NotImplementedError

    def _out(self, data: bytes):
        """Linux okuyorsa baytlar ondan, değilse bu hattan gider."""
        route = self.route
        if route is not None and route(data):
            self.linux_tx += len(data)
        else:
            self._write(data)

    def hold(self):
        """Hattı bir dizi yazım için ayırır (rapor 092): `with link.hold():` içinde başka thread'in
        satırı (ör. ARP cevabı) araya giremez. Yarım bırakılıp bekletilen satırlar
        (T01, T02 satır timeout'u) bunu kullanır: tek seri hatta yarım satırın arasına başka satır
        girerse kart ikisini tek, bozuk bir satır görür."""
        return self._tx_lock

    def write(self, data: bytes):
        with self._tx_lock:
            self._out(data)
            self.tx_bytes += len(data)
            self.tx_lines += data.count(b"\n")
        if self.on_raw_tx is not None:
            self.on_raw_tx(data)

    def write_fragmented(self, data: bytes, mode: str = "whole", seed: int = 1234,
                         min_chunk: int = 1, max_chunk: int = 17, gap_s: float = 0.002):
        """T01: 'whole' tek yazım, 'bytes' bayt bayt, 'random' seed'li 1..17 baytlık parçalar."""
        if mode == "whole":
            self.write(data)
            return
        rng = random.Random(seed)
        i = 0
        with self._tx_lock:
            while i < len(data):
                n = 1 if mode == "bytes" else rng.randint(min_chunk, max_chunk)
                part = data[i:i + n]
                self._out(part)
                i += n
                if gap_s > 0:
                    time.sleep(gap_s)
            self.tx_bytes += len(data)
            self.tx_lines += data.count(b"\n")
        if self.on_raw_tx is not None:
            self.on_raw_tx(data)

    def close(self):
        self.closed = True


class SerialLink(Link):
    def __init__(self, port: str, baud: int = 1000000):
        super().__init__()
        if serial is None:
            raise RuntimeError("pyserial yüklü değil: pip install pyserial")
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(port, baud, timeout=0.05, write_timeout=2.0)
        self.ser.reset_input_buffer()
        self._thread = threading.Thread(target=self._reader, name="serial-rx", daemon=True)
        self._thread.start()

    def _reader(self):
        while not self.closed:
            if not self._reading.is_set():
                time.sleep(0.02)            # okumuyoruz: baytlar OS / USB'de kalır
                continue
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except Exception:
                if self.closed:
                    break
                time.sleep(0.1)
                continue
            if data:
                self._feed(data)

    def _write(self, data: bytes):
        self.ser.write(data)
        self.ser.flush()

    def close(self):
        super().close()
        try:
            self.ser.close()
        except Exception:
            pass

    @property
    def name(self):
        return "%s @ %d" % (self.port, self.baud)


class LinuxLink(Link):
    """Seri portsuz: baytlar yalnız route (Linux) ile gider, satırlar feed_line ile gelir."""

    name = "MP257 (Linux)"

    def __init__(self, on_close=None):
        super().__init__()
        self.dropped = 0                       # Linux yokken yazılan, hiçbir yere gitmeyen bayt
        self.on_close = on_close

    def _write(self, data: bytes):
        self.dropped += len(data)

    def close(self):
        super().close()
        if self.on_close is not None:
            self.on_close()


class SimLink(Link):
    """Simülatöre bağlı hat. Kartın çıkışı kart tarafında baud hızında sıraya girer."""

    def __init__(self, board):
        super().__init__()
        self.board = board
        self._pending = []
        self._pend_lock = threading.Lock()
        board.attach(self._from_board)
        self.name = "simülatör"

    def _from_board(self, data: bytes):
        # Okuma durdurulduysa (T06) baytlar burada birikir; 64 KB'den sonrası düşer
        # (gerçekte USB / OS tamponu ve VCP).
        if not self._reading.is_set():
            with self._pend_lock:
                if sum(len(x) for x in self._pending) < 65536:
                    self._pending.append(data)
            return
        with self._pend_lock:
            pend, self._pending = self._pending, []
        for p in pend:
            self._feed(p)
        self._feed(data)

    def pause_reading(self, paused: bool):
        super().pause_reading(paused)
        if not paused:
            with self._pend_lock:
                pend, self._pending = self._pending, []
            for p in pend:
                self._feed(p)

    def _write(self, data: bytes):
        self.board.feed(data)

    def close(self):
        super().close()
        self.board.stop()
