"""Uçuş (IMU) ve MP257 (UDP) sekmelerinin ortak sayfası: 3B görünüm, açılar, sayaçlar ve tek gecikme tablosu.

İki sayfa da aynı kaynağı (source.TelemSource) okur; seri bağlantı, STOP ve testler sayfayı durdurmaz.
"""

import time
import tkinter as tk
from tkinter import ttk, messagebox

from . import proto, source as src, udp_in
from .flight_view import FlightCanvas, C, FONT_UI, FONT_NUM

SOURCE_TEXT = {src.MP257: ("Kaynak: MP257 (Linux, UDP %d)" % udp_in.PORT, "#2e7d32"),
               src.DIRECT: ("Kaynak: F746 doğrudan (UART), Linux yok", "#b26a00"),
               src.NONE: ("Veri gelmiyor", "#6b7280")}


def sensor_text(t):
    """Jiro, ivme ve yer çekimi çıkarılmış ivme satırları."""
    def row(name, digits, values):
        return "%-12s" % name + " ".join("%s %+8.*f" % (axis, digits, round(v, digits) + 0.0)     # + 0.0: -0 yazılmaz
                                         for axis, v in zip("xyz", values))
    return "\n".join((row("jiro (°/s)", 1, t.gyro_dps), row("ivme (g)", 3, t.accel_g), row("hareket (g)", 3, t.motion_g)))


def cal_status(t, asked):
    """Kalibrasyon durumu: (yazı, zemin, yazı rengi). asked: bu oturumda CAL gönderildi."""
    if t is None:
        return "veri yok", "#e5e7eb", "#374151"
    still = "duruyor" if t.still else "hareketli"
    if t.cal_busy:
        return "oynatmayın…  (%s)" % still, "#FFC21A", "#111827"
    if t.cal_done:
        return "sıfırlandı · " + still, "#2e7d32", "white"
    if asked:
        return "olmadı: kart durmadı", "#b91c1c", "white"
    return "kalibre değil · " + still, "#e5e7eb", "#374151"


class LatencyTable(ttk.LabelFrame):
    """Bütün gecikmeler tek tabloda, µs. İki sayfada da aynı veriden çizilir."""

    COLS = (("min", "En az", 80), ("avg", "Ort", 80), ("max", "En çok", 80), ("jit", "Jitter", 80),
            ("last", "Son", 80), ("n", "Adet", 90))

    def __init__(self, master, source):
        super().__init__(master, text="Gecikme (µs)", padding=8)
        self.source = source
        st = ttk.Style()
        st.configure("Lat.Treeview", font=(FONT_UI, 10), rowheight=26)
        st.configure("Lat.Treeview.Heading", font=(FONT_UI, 10, "bold"))
        self.tree = ttk.Treeview(self, columns=[c[0] for c in self.COLS], show="tree headings", height=5,
                                 selectmode="none", style="Lat.Treeview")
        self.tree.heading("#0", text="Aşama: nereden → nereye", anchor="w")
        self.tree.column("#0", width=470, minwidth=300)
        for key, text, width in self.COLS:
            self.tree.heading(key, text=text, anchor="e")
            self.tree.column(key, width=width, minwidth=60, anchor="e")
        self.tree.tag_configure("alt", background="#f3f5f9")
        self.tree.tag_configure("sum", background="#dff3e6", font=(FONT_UI, 10, "bold"))
        for iid, text, tag in (
                ("f746", "F746: DRDY → USART6 TX bitti", ""),
                ("m33", "MP257: ilk bayt alındı (T1) → Linux’a verildi (T5)", "alt"),
                ("tail", "MP257: son bayt alındı (T2) → Linux’a verildi (T5)", ""),
                ("whole", "TOPLAM: DRDY → Linux’a verildi (T5)", "sum"),
                ("direct", "F746 doğrudan: DRDY → UART TX bitti (Linux yokken)", "alt")):
            self.tree.insert("", "end", iid=iid, text=text, tags=(tag,) if tag else ())
        self.tree.pack(fill="x")
        self.note = tk.StringVar()
        ttk.Label(self, textvariable=self.note, font=(FONT_UI, 9, "bold")).pack(anchor="w", pady=(4, 0))
        ttk.Label(self, text="Toplam = 1. satır + 3. satır (aynı örnek). 2. satır hattaki 27 baytı da içerir, toplanmaz. "
                             "T2, ölçülmüş 563 µs alma süresinden hesaplanır (≈ ±2 µs). Linux’un içi, USB ağı ve PC dahil değil.",
                  style="Small.TLabel").pack(anchor="w")

    def refresh(self):
        s = self.source
        u = s.udp
        for iid, span in (("f746", u and u.f746), ("m33", u and u.m33), ("tail", u and u.tail),
                          ("whole", u and u.whole), ("direct", s.direct)):
            self.tree.item(iid, values=span.cells() if span else ("—",) * 6)
        if u is not None and u.whole.n:
            self.note.set("Toplam süre: hedef ≤ %g ms %.1f %%   ·   deadline > %g ms: %d / %d" % (
                proto.TELEM_TARGET_MS, 100.0 * u.le_target / u.whole.n, proto.TELEM_DEADLINE_MS, u.over_deadline, u.whole.n))
        else:
            self.note.set("Toplam süre: MP257 verisi bekleniyor.")


class ImuPage(ttk.Frame):
    """app: gui.App (.source, .session, .rate_var, ._cmd). detail: True ise son örneğin ayrıntısı (Uçuş), değilse sayaçlar."""

    def __init__(self, master, app, detail):
        super().__init__(master, padding=8)
        self.app = app
        self.source = app.source
        self.detail = detail
        self.drawn = None
        self.columnconfigure(0, weight=3)
        self.columnconfigure(1, weight=1, minsize=330)
        self.rowconfigure(0, weight=1)

        self.view = FlightCanvas(self, width=760, height=320)
        self.view.note = "Veri yok: Başlat'a basın."
        self.view.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        side = ttk.Frame(self)
        side.grid(row=0, column=1, sticky="nsew")

        g = ttk.LabelFrame(side, text="MP257 → PC", padding=6)
        g.pack(fill="x")
        g.columnconfigure((0, 1), weight=1)
        self.src_label = tk.Label(g, fg="white", font=(FONT_UI, 9, "bold"), anchor="w", padx=8, pady=3)
        self.src_label.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 4))
        self.btn_start = ttk.Button(g, text="▶ Başlat", command=self._start)
        self.btn_start.grid(row=1, column=0, sticky="ew", padx=2, pady=2)
        self.btn_stop = ttk.Button(g, text="■ Durdur", command=app.session.stop)
        self.btn_stop.grid(row=1, column=1, sticky="ew", padx=2, pady=2)
        ttk.Button(g, text="Yaw sıfırla (görüntü)", command=self._zero_yaw).grid(row=2, column=0, sticky="ew", padx=2, pady=2)
        ttk.Button(g, text="Kamerayı sıfırla", command=self.view.reset_camera).grid(row=2, column=1, sticky="ew", padx=2,
                                                                                   pady=2)
        ttk.Button(g, text="Kalibre et (CAL)", command=self._calibrate).grid(row=3, column=0, sticky="ew", padx=2, pady=2)
        self.cal_label = tk.Label(g, font=(FONT_UI, 9, "bold"), padx=4, pady=3)
        self.cal_label.grid(row=3, column=1, sticky="ew", padx=2, pady=2)
        fr = ttk.Frame(g)
        fr.grid(row=4, column=0, columnspan=2, sticky="ew", pady=2)
        ttk.Label(fr, text="IMU hızı (Hz)").pack(side="left", padx=2)
        ttk.Combobox(fr, textvariable=app.rate_var, values=proto.RATES_HZ, state="readonly", width=6).pack(side="left")
        ttk.Button(fr, text="Hızı uygula", command=lambda: app._cmd("RATE")).pack(side="left", padx=6)
        self.state_var = tk.StringVar()
        ttk.Label(g, textvariable=self.state_var, style="Small.TLabel", wraplength=310, justify="left").grid(
            row=5, column=0, columnspan=2, sticky="w")

        g = ttk.LabelFrame(side, text="Açılar ve sensörler", padding=6)
        g.pack(fill="x", pady=(6, 0))
        self.ang_vars = {}
        for i, (key, name, col) in enumerate((("roll", "Roll φ", C["roll"]), ("pitch", "Pitch θ", C["pitch"]),
                                              ("yaw", "Yaw ψ", C["yaw"]))):
            tk.Label(g, text="■", fg=col, font=(FONT_UI, 12)).grid(row=i, column=0, sticky="w")
            ttk.Label(g, text=name, font=(FONT_UI, 11, "bold")).grid(row=i, column=1, sticky="w", padx=(2, 10))
            v = tk.StringVar(value="—")
            ttk.Label(g, textvariable=v, font=(FONT_NUM, 18, "bold"), width=9, anchor="e").grid(row=i, column=2, sticky="e")
            self.ang_vars[key] = v
        g.columnconfigure(2, weight=1)
        self.sensor_var = tk.StringVar(value="—")
        ttk.Label(g, textvariable=self.sensor_var, font=(FONT_NUM, 9), justify="left").grid(
            row=3, column=0, columnspan=3, sticky="w", pady=(4, 0))

        g = ttk.LabelFrame(side, text="Son örnek" if detail else "Sayaçlar", padding=6)
        g.pack(fill="x", pady=(6, 0))
        self.info_var = tk.StringVar(value="—")
        ttk.Label(g, textvariable=self.info_var, font=(FONT_NUM, 9), justify="left").pack(anchor="w")

        self.table = LatencyTable(self, self.source)
        self.table.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        self.after(33, self._frame)
        self.after(250, self._panel)

    def _start(self):
        error = self.app.session.start()
        if error:
            messagebox.showerror("MP257", error)

    def _calibrate(self):
        self.drawn = None
        self.app.calibrate()

    def _zero_yaw(self):
        if self.source.latest is not None:
            self.source.yaw_zero = self.source.latest.yaw
            self.drawn = None

    # ------------------------------------------------------------ çizim döngüleri
    def _frame(self):
        t = self.source.latest
        if t is not None and self.winfo_ismapped() and (t, self.source.yaw_zero) != self.drawn:
            yaw = self.source.shown_yaw(t)
            self.view.set_sample(t.roll, t.pitch, yaw, t.accel_g, t.bias_ready)
            self.view.draw()
            self.drawn = (t, self.source.yaw_zero)
            for key, value in (("roll", t.roll), ("pitch", t.pitch), ("yaw", yaw)):
                self.ang_vars[key].set("%+.2f°" % value)
            self.sensor_var.set(sensor_text(t))
        self.after(33, self._frame)

    def _panel(self):
        s, ses = self.source, self.app.session
        text, color = SOURCE_TEXT[s.active()]
        self.src_label.config(text=text, bg=color)
        self.btn_start.config(state="disabled" if ses.wanted else "normal")
        self.btn_stop.config(state="normal" if ses.wanted else "disabled")
        self.state_var.set(s.error or ses.text or "Başlat: kartta (%s) %s ssh ile çalışır. M33 çalışıyor olmalı." % (
            udp_in.BOARD_IP, udp_in.PROGRAM))
        text, bg, fg = cal_status(s.latest if s.active() != src.NONE else None, s.cal_asked)
        self.cal_label.config(text=text, bg=bg, fg=fg)
        if self.winfo_ismapped():
            self.info_var.set(self._detail_text() if self.detail else self._count_text())
            self.table.refresh()
        self.after(250, self._panel)

    def _count_text(self):
        s = self.source
        t = s.stats()
        return "alınan       %d\nanlık hız    %.1f Hz\nSEQ boşluğu  %d\nbozuk paket  %d" % (
            t.received, t.rate_now(time.perf_counter()), t.gaps, s.udp.bad if s.udp is not None else 0)

    def _detail_text(self):
        t = self.source.latest
        if t is None:
            return "—"
        return "SEQ %d   anlık hız %.1f Hz\nbias %s   ivme düzeltmesi %s" % (
            t.seq, self.source.stats().rate_now(time.perf_counter()), "hazır" if t.bias_ready else "ölçülüyor",
            "var" if t.acc_used else "yok")
