"""Arayüzün IMU verisi kaynağı: Linux takılıyken yalnız MP257 (UDP), değilse F746'dan doğrudan (UART).

MP257 paketi geldiği sürece F746'nın UART'tan yolladığı TELEM arayüze verilmez. Son ALIVE_S
saniyede MP257 paketi gelmediyse doğrudan gelen gösterilir.
"""

import time

from . import udp_in
from .hoststack import TelemetryStats

ALIVE_S = 2.0               # MP257'den bu kadar süre paket gelmezse Linux yok sayılır
RETRY_S = 3.0               # karttaki telem_udp kendiliğinden biterse yeniden başlatma aralığı
NONE, MP257, DIRECT = "yok", "MP257", "F746"


class TelemSource:
    def __init__(self, port=udp_in.PORT):
        self.port = port
        self.udp = None                     # UdpTelem; program boyunca açık
        self.lines = None                   # UdpLines: F746'nın Linux üzerinden gelen satırları
        self.on_line = None                 # fonksiyon(bytes)
        self.error = ""                     # port açılamadıysa nedeni
        self.direct_stats = TelemetryStats()
        self.direct = udp_in.Span()         # DRDY -> UART TX bitti (yalnız Linux yokken)
        self.ignored = 0                    # MP257 akarken gösterilmeyen doğrudan TELEM
        self.yaw_zero = 0.0                 # iki sayfada da gösterilen yaw'dan çıkarılır
        self.cal_asked = False              # arayüzden CAL gönderildi (sonuç: Telem.cal_*)
        self.latest = None
        self.listeners = []                 # fonksiyon(Telem): gösterilen
        self.linux_listeners = []           # fonksiyon(Telem): yalnız MP257'den gelen
        self._udp_time = self._direct_time = None
        self.open()

    def open(self):
        if self.udp is None:
            try:
                self.lines = udp_in.UdpLines(self.port + 1, on_line=self._on_line)
                self.udp = udp_in.UdpTelem(self.port, on_telem=self._on_udp)
                self.error = ""
            except OSError as e:
                self.error = "UDP %d / %d açılamadı: %s" % (self.port, self.port + 1, e)
                if self.lines is not None:
                    self.lines.close()
                    self.lines = None
        return self.udp is not None

    def close(self):
        if self.udp is not None:
            self.udp.close()
            self.lines.close()
            self.udp = self.lines = None

    def active(self, now=None):
        now = time.perf_counter() if now is None else now
        if self._udp_time is not None and now - self._udp_time <= ALIVE_S:
            return MP257
        if self._direct_time is not None and now - self._direct_time <= ALIVE_S:
            return DIRECT
        return NONE

    def stats(self):
        """Gösterilen kaynağın sayaçları; hiçbiri akmıyorsa en son akanınki."""
        if self.udp is None or (self._direct_time or 0) > (self._udp_time or 0):
            return self.direct_stats
        return self.udp.telem

    def reset(self):
        if self.udp is not None:
            self.udp.reset()
        self.direct_stats.reset()
        self.direct = udp_in.Span()
        self.ignored = 0

    def shown_yaw(self, t):
        return ((t.yaw - self.yaw_zero + 180.0) % 360.0) - 180.0

    def send_command(self, data):
        """Link.route: Linux okuyorsa baytları ona yollar, True; değilse False (UART'tan gider)."""
        lines = self.lines
        if lines is None or self.active() != MP257:
            return False
        try:
            lines.send(data)
        except OSError:
            return False                    # USB ağı gitti
        return True

    def attach(self, lk):
        """Hattı Linux yoluna bağlar: satırlar buradan gelir, Linux okurken baytlar buradan gider."""
        self.on_line = lk.feed_line
        lk.route = self.send_command
        lk.linux_active = lambda: self.active() == MP257
        lk.on_pause = self.pause

    def pause(self, paused):
        for x in (self.udp, self.lines):
            if x is not None:
                x.paused = paused

    def linux_gone(self):
        """Oturum bitti: ALIVE_S beklenmeden UART'a dönülür."""
        self._udp_time = None

    def _on_line(self, line):
        if self.on_line is not None:
            self.on_line(line)

    def _on_udp(self, t):
        self._udp_time = time.perf_counter()
        self._publish(t)
        for fn in list(self.linux_listeners):
            fn(t)

    def on_direct(self, t):
        """F746'nın UART'tan yolladığı TELEM (Host'un okuma thread'i çağırır)."""
        if self.active() == MP257:
            self.ignored += 1
            return
        self._direct_time = time.perf_counter()
        self.direct_stats.add(t)
        if t.prev_valid:
            self.direct.add(t.prev_us)
        self._publish(t)

    def _publish(self, t):
        self.latest = t                     # tek atama: çizim döngüleri en sonuncuyu okur
        for fn in list(self.listeners):
            fn(t)


class BoardSession:
    """Karttaki telem_udp'nin ssh oturumu; iki sayfanın Başlat / Durdur'u. Durdur'a kadar, biterse yeniden başlar."""

    def __init__(self, source, run=udp_in.run_board):
        self.source = source
        self.run = run
        self.ssh = None
        self.wanted = False
        self.text = ""
        self._retry_at = 0.0

    def start(self):
        """Hata varsa metnini, yoksa boş metin döndürür."""
        if not self.source.open():
            return self.source.error
        self.source.reset()
        error = self._launch()
        self.wanted = not error
        return error

    def _launch(self):
        try:
            self.ssh = self.run(udp_in.pc_ip())
        except OSError as e:                # ağ yok ya da ssh kurulu değil
            self.text = "Başlatılamadı: %s" % e
            return self.text
        self.text = "Dinleniyor; telem_udp kartta çalışıyor."
        return ""

    def stop(self):
        self.wanted = False
        if self.ssh is not None:
            self.ssh.terminate()            # ssh kopar: kartta telem_udp M33'e '0' yollar ve kapanır
            self.ssh.wait()
            self.ssh = None
        self.source.linux_gone()
        self.text = "Durduruldu."

    def ensure(self, timeout=8.0):
        """Oturum yoksa başlatır, MP257 verisini bekler; gelirse True. Arka plan thread'inden çağrılır."""
        if not self.wanted and self.source.active() != MP257:
            self.start()
        end = time.perf_counter() + timeout
        while time.perf_counter() < end:
            if self.source.active() == MP257:
                return True
            time.sleep(0.05)
        return False

    def poll(self, now=None):
        """Düzenli çağrılır: kendiliğinden biten oturumu RETRY_S sonra yeniden başlatır."""
        now = time.perf_counter() if now is None else now
        if not self.wanted:
            return
        if self.ssh is not None and self.ssh.poll() is not None:
            out = self.ssh.stdout.read().strip()
            self.text = "telem_udp kapandı (çıkış %d): %s. Yeniden denenecek." % (self.ssh.returncode, out or "çıktı yok")
            self.ssh = None
            self._retry_at = now + RETRY_S
        elif self.ssh is None and now >= self._retry_at:
            self._retry_at = now + RETRY_S
            self._launch()
