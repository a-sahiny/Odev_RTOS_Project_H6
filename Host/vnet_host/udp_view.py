"""MP257 (UDP) sekmesi (M8): kartta telem_udp'yi ssh ile çalıştırır, UDP 5002'den gelen TELEM'in
açılarını canlı gösterir. Seri hattan (Karta bağlan) bağımsızdır."""

import tkinter as tk
from tkinter import ttk, messagebox

from . import udp_in
from .flight_view import FlightCanvas, C, FONT_UI, FONT_NUM


class LatencyPanel(ttk.LabelFrame):
    """MP257 yolunun gecikmeleri, canlı. source() o anki UdpTelem'i ya da None döndürür; Başlat'ta sıfırlanır."""

    def __init__(self, master, source):
        super().__init__(master, text="Gecikme: MP257 yolu (µs)", padding=8)
        self.source = source
        self.var = tk.StringVar(value="Veri yok: MP257 (UDP) sekmesinde Başlat.")
        ttk.Label(self, textvariable=self.var, font=(FONT_NUM, 9), justify="left").pack(anchor="w")
        ttk.Label(self, text="F746: DRDY → USART6 son bit. MP257: ilk bayt kesmesi (T1) → Linux’a verildi (T5). "
                             "İkisi de 23 baytın hattaki ≈ 0,5 ms’sini içerir; toplanmaz.",
                  style="Small.TLabel", wraplength=310, justify="left").pack(anchor="w", pady=(4, 0))
        self.after(500, self._tick)

    def _tick(self):
        rx = self.source()
        if rx is not None:                  # durdurulunca son değerler kalır
            self.var.set("%-5s %6s %6s %6s %6s %6s\n%s\n%s" % ("", "en az", "ort", "en çok", "jitter", "adet",
                                                               rx.f746.row("F746"), rx.m33.row("MP257")))
        self.after(500, self._tick)


class UdpTab(ttk.Frame):
    def __init__(self, master):
        super().__init__(master, padding=8)
        self.rx = None                      # UdpTelem
        self.ssh = None                     # karttaki telem_udp'nin ssh süreci
        self.latest = None                  # son Telem (alma thread'i yazar)
        self.drawn = None
        self.columnconfigure(0, weight=3)
        self.columnconfigure(1, weight=1, minsize=330)
        self.rowconfigure(0, weight=1)

        self.view = FlightCanvas(self, width=760, height=560)
        self.view.note = "Veri yok: Başlat'a basın."
        self.view.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        side = ttk.Frame(self)
        side.grid(row=0, column=1, sticky="nsew")

        g = ttk.LabelFrame(side, text="MP257 → PC (UDP %d)" % udp_in.PORT, padding=8)
        g.pack(fill="x")
        g.columnconfigure((0, 1), weight=1)
        self.btn_start = ttk.Button(g, text="▶ Başlat", command=self.start)
        self.btn_start.grid(row=0, column=0, sticky="ew", padx=2, pady=2)
        self.btn_stop = ttk.Button(g, text="■ Durdur", command=self.stop, state="disabled")
        self.btn_stop.grid(row=0, column=1, sticky="ew", padx=2, pady=2)
        ttk.Label(g, text="Başlat: kartta (%s) %s ssh ile çalışır, PC UDP'yi dinler. M33 çalışıyor olmalı." % (
            udp_in.BOARD_IP, udp_in.PROGRAM), style="Small.TLabel", wraplength=300).grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(4, 0))

        g = ttk.LabelFrame(side, text="Açılar", padding=8)
        g.pack(fill="x", pady=(8, 0))
        self.ang_vars = {}
        for i, (key, name, col) in enumerate((("roll", "Roll φ", C["roll"]), ("pitch", "Pitch θ", C["pitch"]),
                                              ("yaw", "Yaw ψ", C["yaw"]))):
            tk.Label(g, text="■", fg=col, font=(FONT_UI, 12)).grid(row=i, column=0, sticky="w")
            ttk.Label(g, text=name, font=(FONT_UI, 11, "bold")).grid(row=i, column=1, sticky="w", padx=(2, 10))
            v = tk.StringVar(value="—")
            ttk.Label(g, textvariable=v, font=(FONT_NUM, 20, "bold"), width=9, anchor="e").grid(row=i, column=2, sticky="e")
            self.ang_vars[key] = v
        g.columnconfigure(2, weight=1)

        g = ttk.LabelFrame(side, text="Sayaçlar", padding=8)
        g.pack(fill="x", pady=(8, 0))
        self.cnt_var = tk.StringVar(value="—")
        ttk.Label(g, textvariable=self.cnt_var, font=(FONT_NUM, 10), justify="left").pack(anchor="w")
        LatencyPanel(side, lambda: self.rx).pack(fill="x", pady=(8, 0))
        self.state_var = tk.StringVar(value="")
        ttk.Label(side, textvariable=self.state_var, wraplength=320, justify="left").pack(anchor="w", pady=(8, 0))

        self.winfo_toplevel().protocol("WM_DELETE_WINDOW", self._close)
        self.after(33, self._frame)
        self.after(500, self._panel)

    # ------------------------------------------------------------ düğmeler
    def start(self):
        try:
            self.rx = udp_in.UdpTelem(on_telem=self._on_telem)
            self.ssh = udp_in.run_board(udp_in.pc_ip())
        except OSError as e:                # port kullanımda, ağ yok ya da ssh kurulu değil
            if self.rx is not None:
                self.rx.close()
            self.rx = None
            messagebox.showerror("MP257 UDP", "Başlatılamadı: %s" % e)
            return
        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")
        self.state_var.set("Dinleniyor; telem_udp kartta çalışıyor.")

    def stop(self, why="Durduruldu."):
        if self.ssh is not None:
            self.ssh.terminate()            # ssh kopar: kartta telem_udp M33'e '0' yollar ve kapanır
            self.ssh.wait()
        if self.rx is not None:
            self._show()                    # son sayılar ekranda kalsın
            self.rx.close()
        self.ssh = self.rx = None
        self.btn_start.config(state="normal")
        self.btn_stop.config(state="disabled")
        self.state_var.set(why)

    def _close(self):
        self.stop()
        self.winfo_toplevel().destroy()

    def _on_telem(self, t):
        self.latest = t                     # tek atama: çizim döngüsü en sonuncuyu okur

    # ------------------------------------------------------------ çizim döngüleri
    def _frame(self):
        t = self.latest
        if t is not None and t is not self.drawn and self.winfo_ismapped():
            self.view.set_sample(t.roll, t.pitch, t.yaw, t.accel_g, t.bias_ready)
            self.view.draw()
            self.drawn = t
            for key in ("roll", "pitch", "yaw"):
                self.ang_vars[key].set("%+.2f°" % getattr(t, key))
        self.after(33, self._frame)

    def _show(self):
        t = self.rx.telem
        self.cnt_var.set("alınan       %d\nhız          %.1f Hz\nSEQ boşluğu  %d\nbozuk paket  %d" % (
            t.received, t.rate_hz(), t.gaps, self.rx.bad))

    def _panel(self):
        if self.rx is not None:
            self._show()
            if self.ssh.poll() is not None:     # telem_udp kendiliğinden bitti (ör. M33 çalışmıyor)
                self.stop("telem_udp kapandı (çıkış %d): %s" % (self.ssh.returncode, self.ssh.stdout.read().strip()))
        self.after(500, self._panel)
