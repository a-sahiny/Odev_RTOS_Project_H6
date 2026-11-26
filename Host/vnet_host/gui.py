"""Grafik arayüz (tkinter, Python ile gelir; ek paket gerekmez).

Sekmeler: Kontrol, Uçuş (IMU), MP257 (UDP), Telemetri, Kaynaklar, Sayaçlar, Testler, Günlük.
Host yığınının olayları (okuma thread'inden) bir queue ile arayüz thread'ine taşınır.
"""

import collections
import json
import os
import queue
import random
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

from . import accounting as acc
from . import ethframe as eth
from . import link as linkmod
from . import proto
from . import source as src
from .hoststack import Host, HostError
from . import FROZEN, HOST_DIR
from .stats_ids import COUNTER_NAMES

COL_OK = "#1e7d32"
COL_BAD = "#c62828"
COL_WARN = "#ef6c00"
COL_IDLE = "#757575"
COL_BG = "#fafafa"
PLOT_COLORS = ("#d32f2f", "#388e3c", "#1976d2")

TASK_NAMES = ("critParser", "ctrlTask", "vnetRx", "uartTx", "meas", "lowParser", "worker", "IP-task", "idle")


class Plot(tk.Canvas):
    """Üç kanallı kayan çizgi grafik."""

    def __init__(self, master, title, unit, scale, **kw):
        super().__init__(master, background="white", highlightthickness=1, highlightbackground="#bbb", **kw)
        self.title = title
        self.unit = unit
        self.scale = scale
        self.data = [collections.deque(maxlen=400) for _ in range(3)]
        self.bind("<Configure>", lambda e: self.redraw())

    def add(self, values):
        for d, v in zip(self.data, values):
            d.append(v / self.scale)

    def clear(self):
        for d in self.data:
            d.clear()
        self.redraw()

    def redraw(self):
        self.delete("all")
        w, hgt = self.winfo_width(), self.winfo_height()
        if w < 50 or hgt < 40:
            return
        left, top, bottom = 48, 22, hgt - 18
        vals = [v for d in self.data for v in d]
        lo, hi = (min(vals), max(vals)) if vals else (-1.0, 1.0)
        if hi - lo < 1e-6:
            hi, lo = hi + 0.5, lo - 0.5
        pad = (hi - lo) * 0.08
        lo, hi = lo - pad, hi + pad
        self.create_text(left, 4, text="%s (%s)" % (self.title, self.unit), anchor="nw", font=("Segoe UI", 9, "bold"))
        for i in range(5):
            y = top + (bottom - top) * i / 4
            v = hi - (hi - lo) * i / 4
            self.create_line(left, y, w - 6, y, fill="#eee")
            self.create_text(left - 4, y, text="%.2f" % v, anchor="e", font=("Segoe UI", 8), fill="#666")
        names = ("x", "y", "z")
        for k, (d, col) in enumerate(zip(self.data, PLOT_COLORS)):
            self.create_text(w - 90 + 28 * k, 4, text=names[k], anchor="nw", fill=col, font=("Segoe UI", 9, "bold"))
            if len(d) < 2:
                continue
            n = d.maxlen
            pts = []
            for i, v in enumerate(d):
                x = left + (w - 6 - left) * (i + n - len(d)) / (n - 1)
                y = top + (bottom - top) * (hi - v) / (hi - lo)
                pts += [x, y]
            self.create_line(*pts, fill=col, width=1.4)


AUTO_PORT = "Otomatik (Linux, yoksa F746)"
CAL_TRIES = 4               # kart üst üste 3 eski SEQ'ten sonra yenisini kabul eder (APP_SEQ_RESYNC_COUNT)


class App:
    def __init__(self, root):
        self.root = root
        root.title("VNET Telemetri İstasyonu — host aracı")
        root.geometry("1180x820")
        root.minsize(1100, 820)
        self.h = None
        self.events = queue.Queue()
        self.source = src.TelemSource()          # IMU verisi: MP257 (UDP); Linux yoksa F746 (UART)
        self.source.listeners.append(lambda t: self.events.put(("telem", t)))
        self.source.on_line = self._linux_line
        self.source.linux_listeners.append(self._linux_telem)
        self.session = src.BoardSession(self.source)
        self.busy = False
        self.gen_running = False
        self.ref_counters = None
        self.last_counters = None
        self.log_lines = collections.deque(maxlen=3000)
        self._style()
        self._build()
        self.root.after(60, self._pump)
        self.root.after(1000, self._tick)
        self.root.after(250, self._poll_session)
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self._refresh_ports()

    # ------------------------------------------------------------ görünüm
    def _style(self):
        st = ttk.Style()
        try:
            st.theme_use("vista" if sys.platform == "win32" else "clam")
        except tk.TclError:
            st.theme_use("clam")
        st.configure("Chip.TLabel", padding=(8, 2), font=("Segoe UI", 9, "bold"), foreground="white")
        st.configure("H.TLabel", font=("Segoe UI", 10, "bold"))
        st.configure("Big.TLabel", font=("Segoe UI", 16, "bold"))
        st.configure("Small.TLabel", font=("Segoe UI", 8), foreground="#555")

    def _build(self):
        top = ttk.Frame(self.root, padding=(10, 8))
        top.pack(fill="x")
        ttk.Label(top, text="Port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=34, state="readonly")
        self.port_cb.pack(side="left", padx=(4, 2))
        ttk.Button(top, text="↻", width=3, command=self._refresh_ports).pack(side="left")
        ttk.Label(top, text="Baud").pack(side="left", padx=(10, 2))
        self.baud_var = tk.StringVar(value="1000000")
        ttk.Combobox(top, textvariable=self.baud_var, values=("1000000", "625000", "115200"), width=8).pack(side="left")
        self.btn_conn = ttk.Button(top, text="Karta bağlan", command=self._connect_serial)
        self.btn_conn.pack(side="left", padx=(10, 2))
        self.btn_sim = ttk.Button(top, text="Simülatör", command=self._connect_sim)
        self.btn_sim.pack(side="left", padx=2)
        self.btn_disc = ttk.Button(top, text="Bağlantıyı kes", command=self._disconnect, state="disabled")
        self.btn_disc.pack(side="left", padx=2)

        self.chips = {}
        chipbar = ttk.Frame(top)
        chipbar.pack(side="right")
        for key, text in (("link", "BAĞLI DEĞİL"), ("net", "NET_READY"), ("meas", "MEAS_EN"), ("fault", "FAULT")):
            lb = tk.Label(chipbar, text=text, bg=COL_IDLE, fg="white", padx=8, pady=2, font=("Segoe UI", 9, "bold"))
            lb.pack(side="left", padx=2)
            self.chips[key] = lb

        info = ttk.Frame(self.root, padding=(10, 0, 10, 6))
        info.pack(fill="x")
        self.info_var = tk.StringVar(value="Kart: — | MAC: — | EPOCH: — | fw: —")
        ttk.Label(info, textvariable=self.info_var).pack(side="left")
        self.linkstat_var = tk.StringVar(value="")
        ttk.Label(info, textvariable=self.linkstat_var, style="Small.TLabel").pack(side="right")

        self.nb = ttk.Notebook(self.root)
        self.nb.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self._tab_control()
        self._tab_flight()
        self._tab_udp()
        self._tab_telem()
        self._tab_resources()
        self._tab_counters()
        self._tab_tests()
        self._tab_log()
        self.status_var = tk.StringVar(value="Bir porta bağlanın ya da Simülatör ile deneyin.")
        ttk.Label(self.root, textvariable=self.status_var, relief="sunken", anchor="w", padding=(8, 2)).pack(fill="x")

    def _tab_control(self):
        f = ttk.Frame(self.nb, padding=10)
        self.nb.add(f, text="Kontrol")
        f.columnconfigure((0, 1, 2), weight=1, uniform="c")

        g = ttk.LabelFrame(f, text="Konsol ve ağ", padding=10)
        g.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        for i, (t, fn) in enumerate((("HELLO", self._do_hello), ("STATE", self._do_state), ("Ping ×4", self._do_ping),
                                     ("ARP çöz", self._do_arp))):
            ttk.Button(g, text=t, command=fn).grid(row=i // 2, column=i % 2, sticky="ew", padx=3, pady=3)
        g.columnconfigure((0, 1), weight=1)
        self.arp_mute_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(g, text="Kartın ARP isteklerine cevap verme (T07)", variable=self.arp_mute_var,
                        command=self._arp_mute).grid(row=2, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Label(g, text="Host → kart FRAME biçimi").grid(row=3, column=0, columnspan=2, sticky="w", pady=(8, 0))
        self.fcs_var = tk.StringVar(value="1")
        fr = ttk.Frame(g)
        fr.grid(row=4, column=0, columnspan=2, sticky="w")
        for val, txt in (("none", "FRAME <hex>"), ("0", "FRAME 0"), ("1", "FRAME 1 (+FCS)")):
            ttk.Radiobutton(fr, text=txt, value=val, variable=self.fcs_var, command=self._fcs).pack(side="left", padx=2)

        g = ttk.LabelFrame(f, text="Komutlar (UDP 5000, önemli lane)", padding=10)
        g.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        g.columnconfigure((0, 1), weight=1)
        ttk.Button(g, text="▶ START", command=lambda: self._cmd("START")).grid(row=0, column=0, sticky="ew", padx=3, pady=3)
        ttk.Button(g, text="■ STOP", command=lambda: self._cmd("STOP")).grid(row=0, column=1, sticky="ew", padx=3, pady=3)
        ttk.Label(g, text="IMU / telemetri hızı (Hz): her IMU örneği bir TELEM").grid(row=1, column=0, columnspan=2,
                                                                                    sticky="w", pady=(8, 0))
        self.rate_var = tk.IntVar(value=proto.RATE_DEFAULT_HZ)
        rr = ttk.Frame(g)
        rr.grid(row=2, column=0, columnspan=2, sticky="ew")
        ttk.Combobox(rr, textvariable=self.rate_var, values=proto.RATES_HZ, state="readonly",
                     width=6).pack(side="left")
        ttk.Button(g, text="SET_RATE uygula", command=lambda: self._cmd("RATE")).grid(row=3, column=0, columnspan=2,
                                                                                    sticky="ew", padx=3, pady=3)
        ttk.Button(g, text="GET_STATS (Sayaçlar)", command=self._read_counters).grid(row=4, column=0, columnspan=2,
                                                                                    sticky="ew", padx=3, pady=3)
        self.resp_var = tk.StringVar(value="Son cevap: —")
        ttk.Label(g, textvariable=self.resp_var, wraplength=330).grid(row=5, column=0, columnspan=2, sticky="w", pady=(8, 0))

        g = ttk.LabelFrame(f, text="Yapay enjeksiyon (konsol)", padding=10)
        g.grid(row=0, column=2, sticky="nsew", padx=4, pady=4)
        g.columnconfigure(1, weight=1)
        self.hold_var = tk.IntVar(value=5000)
        self.txd_var = tk.IntVar(value=50)
        ttk.Label(g, text="HOLD (ms, 0–10000)").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(g, from_=0, to=10000, increment=500, textvariable=self.hold_var, width=8).grid(row=0, column=1, sticky="w")
        ttk.Button(g, text="Gönder", command=lambda: self._console("HOLD %d" % self.hold_var.get())).grid(row=0, column=2)
        ttk.Label(g, text="TXDELAY (ms, 0–1000)").grid(row=1, column=0, sticky="w", pady=4)
        ttk.Spinbox(g, from_=0, to=1000, increment=10, textvariable=self.txd_var, width=8).grid(row=1, column=1, sticky="w")
        ttk.Button(g, text="Gönder", command=lambda: self._console("TXDELAY %d" % self.txd_var.get())).grid(row=1, column=2)
        ttk.Button(g, text="Enjeksiyonu kaldır (TXDELAY 0, HOLD 0)",
                   command=self._clear_injection).grid(row=2, column=0, columnspan=3, sticky="ew", pady=(6, 0))
        ttk.Label(g, text="HOLD lowParser'ı, TXDELAY uartTx'i bekletir. Etkin enjeksiyon raporlarda 'inj' ile görünür.",
                  wraplength=320, style="Small.TLabel").grid(row=3, column=0, columnspan=3, sticky="w", pady=(8, 0))

        g = ttk.LabelFrame(f, text="Önemsiz veri üreteci (UDP 5001, düşük öncelik)", padding=10)
        g.grid(row=1, column=0, columnspan=2, sticky="nsew", padx=4, pady=4)
        self.gen_vars = {k: tk.DoubleVar(value=v) for k, v in (("env", 5), ("bulk", 5), ("cal", 1))}
        for i, (k, txt) in enumerate((("env", "ENV paket/s"), ("bulk", "BULK paket/s"), ("cal", "CAL paket/s"))):
            ttk.Label(g, text=txt).grid(row=0, column=2 * i, sticky="e", padx=(8, 2))
            ttk.Spinbox(g, from_=0, to=50, increment=1, textvariable=self.gen_vars[k], width=5).grid(row=0, column=2 * i + 1)
        self.btn_gen = ttk.Button(g, text="Üreteci başlat", command=self._toggle_gen)
        self.btn_gen.grid(row=0, column=6, padx=10)
        self.gen_var = tk.StringVar(value="gönderilen: —")
        ttk.Label(g, textvariable=self.gen_var).grid(row=1, column=0, columnspan=7, sticky="w", pady=(6, 0))
        ttk.Label(g, text="Tek CAL (ax ay az gx gy gz, ham LSB, |değer| ≤ 4096):").grid(row=2, column=0, columnspan=5,
                                                                                     sticky="w", pady=(10, 0))
        self.cal_var = tk.StringVar(value="0 0 0 0 0 0")
        ttk.Entry(g, textvariable=self.cal_var, width=28).grid(row=2, column=5, columnspan=1, sticky="w", pady=(10, 0))
        ttk.Button(g, text="CAL gönder", command=self._send_cal).grid(row=2, column=6, pady=(10, 0))

        g = ttk.LabelFrame(f, text="Son raporlar", padding=10)
        g.grid(row=1, column=2, sticky="nsew", padx=4, pady=4)
        self.rep_text = tk.Text(g, height=9, width=44, font=("Consolas", 8), wrap="word", relief="flat", bg=COL_BG)
        self.rep_text.pack(fill="both", expand=True)
        f.rowconfigure(1, weight=1)

    def _tab_flight(self):
        from .imu_page import ImuPage
        self.flight = ImuPage(self.nb, self, detail=True)
        self.nb.add(self.flight, text="Uçuş (IMU)")

    def _tab_udp(self):
        from .imu_page import ImuPage
        self.udp = ImuPage(self.nb, self, detail=False)
        self.nb.add(self.udp, text="MP257 (UDP)")

    def _tab_telem(self):
        f = ttk.Frame(self.nb, padding=10)
        self.nb.add(f, text="Telemetri")
        bar = ttk.Frame(f)
        bar.pack(fill="x")
        self.tel_vars = {k: tk.StringVar(value="—") for k in ("rx", "hz", "gap", "lat", "last")}
        for k, txt in (("rx", "Alınan"), ("hz", "Anlık hız (Hz)"), ("gap", "Kayıp (SEQ boşluğu)"),
                       ("lat", "Gecikme son / en çok (ms)")):
            box = ttk.Frame(bar, padding=(0, 0, 24, 0))
            box.pack(side="left")
            ttk.Label(box, text=txt, style="Small.TLabel").pack(anchor="w")
            ttk.Label(box, textvariable=self.tel_vars[k], style="Big.TLabel").pack(anchor="w")
        ttk.Button(bar, text="Sıfırla", command=self._telem_reset).pack(side="right")
        ttk.Label(f, textvariable=self.tel_vars["last"], font=("Consolas", 9)).pack(fill="x", pady=(6, 4))
        self.plot_a = Plot(f, "İvme", "g", proto.ACCEL_LSB_PER_G, height=230)
        self.plot_a.pack(fill="both", expand=True, pady=3)
        self.plot_g = Plot(f, "Jiroskop", "°/s", proto.GYRO_LSB_PER_DPS, height=230)
        self.plot_g.pack(fill="both", expand=True, pady=3)

    def _tab_resources(self):
        f = ttk.Frame(self.nb, padding=10)
        self.nb.add(f, text="Kaynaklar")
        ttk.Label(f, text="Kartın her saniye gönderdiği 'st' raporu (tasarım 18.3). Boştaki seviyeler sabit kalmalı.",
                  style="Small.TLabel").pack(anchor="w")
        cols = ("deger", "aciklama")
        self.res_tree = ttk.Treeview(f, columns=cols, show="tree headings", height=22)
        self.res_tree.heading("#0", text="Alan")
        self.res_tree.heading("deger", text="Değer")
        self.res_tree.heading("aciklama", text="Anlamı")
        self.res_tree.column("#0", width=200)
        self.res_tree.column("deger", width=220)
        self.res_tree.column("aciklama", width=600)
        self.res_tree.pack(fill="both", expand=True, pady=6)
        self.res_tree.tag_configure("warn", foreground=COL_WARN)
        self.res_age_var = tk.StringVar(value="")
        ttk.Label(f, textvariable=self.res_age_var, style="Small.TLabel").pack(anchor="w")

    def _tab_counters(self):
        f = ttk.Frame(self.nb, padding=10)
        self.nb.add(f, text="Sayaçlar")
        bar = ttk.Frame(f)
        bar.pack(fill="x")
        ttk.Button(bar, text="Oku (GET_STATS 0–2)", command=self._read_counters).pack(side="left")
        ttk.Button(bar, text="Referans al", command=self._set_reference).pack(side="left", padx=4)
        ttk.Button(bar, text="Eşitlikleri kontrol et", command=self._check_equalities).pack(side="left")
        self.only_nonzero = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="Yalnız sıfır olmayanlar", variable=self.only_nonzero,
                        command=self._show_counters).pack(side="left", padx=10)
        self.cnt_info = tk.StringVar(value="Δ = referanstan bu yana. Eşitlikler sakin anda tam tutar: STOP, HOLD yok, ≥ 300 ms bekle.")
        ttk.Label(bar, textvariable=self.cnt_info, style="Small.TLabel").pack(side="left", padx=8)
        pw = ttk.PanedWindow(f, orient="horizontal")
        pw.pack(fill="both", expand=True, pady=6)
        left = ttk.Frame(pw)
        self.cnt_tree = ttk.Treeview(left, columns=("id", "name", "val", "delta"), show="headings")
        self.cnt_tree.heading("name", text="Sayaç")
        self.cnt_tree.column("name", width=200, anchor="w")
        for c, t, w in (("id", "#", 40), ("val", "Değer", 110), ("delta", "Δ", 90)):
            self.cnt_tree.heading(c, text=t)
            self.cnt_tree.column(c, width=w, anchor="e")
        sb = ttk.Scrollbar(left, orient="vertical", command=self.cnt_tree.yview)
        self.cnt_tree.configure(yscrollcommand=sb.set)
        self.cnt_tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="left", fill="y")
        pw.add(left, weight=1)
        right = ttk.Frame(pw)
        self.eq_tree = ttk.Treeview(right, columns=("layer", "lhs", "rhs"), show="tree headings")
        self.eq_tree.heading("#0", text="Eşitlik (tasarım 18.2)")
        self.eq_tree.heading("layer", text="Katman")
        self.eq_tree.heading("lhs", text="Sol")
        self.eq_tree.heading("rhs", text="Sağ")
        self.eq_tree.column("#0", width=420)
        for c in ("layer", "lhs", "rhs"):
            self.eq_tree.column(c, width=60, anchor="e")
        self.eq_tree.tag_configure("ok", foreground=COL_OK)
        self.eq_tree.tag_configure("bad", foreground=COL_BAD)
        self.eq_tree.pack(fill="both", expand=True)
        pw.add(right, weight=1)

    def _tab_tests(self):
        import tests
        f = ttk.Frame(self.nb, padding=10)
        self.nb.add(f, text="Testler")
        top = ttk.Frame(f)
        top.pack(fill="x")
        self.test_vars = {}
        grid = ttk.LabelFrame(top, text="Kabul testleri (tasarım 20.2)", padding=8)
        grid.pack(side="left", fill="x", expand=True)
        for i, (tid, (_, title, what)) in enumerate(tests.TESTS.items()):
            v = tk.BooleanVar(value=tid != "T07")
            self.test_vars[tid] = v
            ttk.Checkbutton(grid, text="%s  %s" % (tid, title), variable=v).grid(row=i, column=0, sticky="w")
            ttk.Label(grid, text=what, style="Small.TLabel").grid(row=i, column=1, sticky="w", padx=8)
        opt = ttk.LabelFrame(top, text="Ayarlar", padding=8)
        opt.pack(side="left", fill="y", padx=8)
        self.t04_s = tk.IntVar(value=10)
        self.t07_m = tk.DoubleVar(value=30)
        ttk.Label(opt, text="T04 seviye süresi (s)").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(opt, from_=3, to=120, textvariable=self.t04_s, width=6).grid(row=0, column=1)
        ttk.Label(opt, text="T07 süresi (dk)").grid(row=1, column=0, sticky="w", pady=4)
        ttk.Spinbox(opt, from_=1, to=600, textvariable=self.t07_m, width=6).grid(row=1, column=1)
        self.btn_run = ttk.Button(opt, text="Seçilenleri çalıştır", command=self._run_tests)
        self.btn_run.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        self.test_status = tk.StringVar(value="")
        ttk.Label(opt, textvariable=self.test_status, wraplength=200).grid(row=3, column=0, columnspan=2, sticky="w")
        self.test_tree = ttk.Treeview(f, columns=("exp", "obs", "res"), show="tree headings")
        self.test_tree.heading("#0", text="Girdi")
        self.test_tree.heading("exp", text="Beklenen")
        self.test_tree.heading("obs", text="Gözlenen")
        self.test_tree.heading("res", text="Sonuç")
        self.test_tree.column("#0", width=330)
        self.test_tree.column("exp", width=300)
        self.test_tree.column("obs", width=400)
        self.test_tree.column("res", width=70, anchor="center")
        self.test_tree.tag_configure("GEÇTİ", foreground=COL_OK)
        self.test_tree.tag_configure("KALDI", foreground=COL_BAD)
        self.test_tree.tag_configure("BİLGİ", foreground="#455a64")
        self.test_tree.tag_configure("head", font=("Segoe UI", 9, "bold"))
        self.test_tree.pack(fill="both", expand=True, pady=(8, 0))

    def _tab_log(self):
        f = ttk.Frame(self.nb, padding=10)
        self.nb.add(f, text="Günlük")
        bar = ttk.Frame(f)
        bar.pack(fill="x")
        self.log_frames = tk.BooleanVar(value=False)
        self.log_pause = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="Frame satırlarını da göster", variable=self.log_frames).pack(side="left")
        ttk.Checkbutton(bar, text="Duraklat", variable=self.log_pause).pack(side="left", padx=10)
        ttk.Button(bar, text="Temizle", command=lambda: self.log_text.delete("1.0", "end")).pack(side="left")
        ttk.Label(bar, text="Ham satır gönder:").pack(side="left", padx=(20, 4))
        self.raw_var = tk.StringVar(value="HELLO")
        e = ttk.Entry(bar, textvariable=self.raw_var, width=40)
        e.pack(side="left")
        e.bind("<Return>", lambda ev: self._send_raw())
        ttk.Button(bar, text="Gönder", command=self._send_raw).pack(side="left", padx=4)
        self.log_text = tk.Text(f, font=("Consolas", 9), wrap="none", bg=COL_BG)
        self.log_text.pack(fill="both", expand=True, pady=6)
        self.log_text.tag_configure("tx", foreground="#1565c0")
        self.log_text.tag_configure("rx", foreground="#2e7d32")
        self.log_text.tag_configure("err", foreground=COL_BAD)
        self.log_text.tag_configure("info", foreground="#6a1b9a")

    # ------------------------------------------------------------ bağlantı
    def _refresh_ports(self):
        ports = linkmod.list_ports()
        items = [AUTO_PORT] + ["%s — %s" % p for p in ports]
        self.port_cb["values"] = items
        if self.port_var.get() not in items:
            self.port_var.set(AUTO_PORT)

    def _make_host(self, lk):
        flag = {"none": None, "0": 0, "1": 1}[self.fcs_var.get()]
        h = Host(lk, fcs_flag=flag, log=lambda d, t: self.events.put(("log", (d, t))))
        h.listeners.append(lambda k, d: self._host_event(k, d))
        lk.on_raw_tx = None
        return h

    def _connect_serial(self):
        auto = self.port_var.get() == AUTO_PORT
        sel = linkmod.guess_stlink_port() if auto else self.port_var.get().split(" — ")[0].strip()
        lk = linkmod.LinuxLink()
        if sel:
            try:
                lk = linkmod.SerialLink(sel, int(self.baud_var.get()))
            except Exception as e:
                if not auto:                    # Otomatik'te Linux tek başına da yeter
                    messagebox.showerror("Port açılamadı", str(e))
                    return
        self.source.attach(lk)
        self.source.on_line = self._linux_line       # hat değişse de satır o anki hatta gider
        self._attach(self._make_host(lk), auto)

    def _connect_sim(self):
        from .sim_board import SimBoard
        self._attach(self._make_host(linkmod.SimLink(SimBoard(seed=random.randrange(1 << 16)))))

    def _attach(self, h, auto=False):
        self._disconnect()
        self.h = h
        self.btn_disc.config(state="normal")
        self.btn_conn.config(state="disabled")
        self.btn_sim.config(state="disabled")
        self._link_chip()
        self._bg("Bağlanılıyor (HELLO + ARP)…", self._hello_and_arp, auto)

    def _disconnect(self):
        self.gen_running = False
        if self.h is not None:
            try:
                self.h.link.close()
            except Exception:
                pass
        self.h = None
        self.btn_disc.config(state="disabled")
        self.btn_conn.config(state="normal")
        self.btn_sim.config(state="normal")
        self._chip("link", "BAĞLI DEĞİL", COL_IDLE)
        for k in ("net", "meas", "fault"):
            self._chip(k, self.chips[k].cget("text"), COL_IDLE)

    # ------------------------------------------------------------ arka plan işleri
    def _bg(self, label, fn, *args):
        if self.h is None:
            messagebox.showinfo("Bağlantı yok", "Önce karta ya da simülatöre bağlanın.")
            return
        if self.busy:
            self.status_var.set("Meşgul: önceki iş bitmedi.")
            return
        self.busy = True
        self.status_var.set(label)

        def work():
            try:
                out = fn(*args)
                self.events.put(("done", (label, out)))
            except HostError as e:
                self.events.put(("error", str(e)))
            except Exception as e:      # arayüz düşmesin
                self.events.put(("error", "%s: %s" % (type(e).__name__, e)))
            finally:
                self.busy = False

        threading.Thread(target=work, daemon=True).start()

    def _link_chip(self):
        """Bağlantının o anki yolu: Linux akıyorsa o, değilse seri port."""
        lk = self.h.link
        if lk.route is not None and self.source.active() == src.MP257:
            self._chip("link", "BAĞLI: MP257 (Linux)", COL_OK)
        elif isinstance(lk, linkmod.LinuxLink):
            self._chip("link", "LİNUX VERİSİ YOK", COL_WARN)
        else:
            self._chip("link", "BAĞLI: " + getattr(lk, "name", "?"), COL_OK)

    def _hello_and_arp(self, auto=False):
        if auto and not self.session.ensure() and isinstance(self.h.link, linkmod.LinuxLink):
            raise HostError("Linux'tan veri gelmiyor ve F746'nın seri portu yok: %s" % (
                self.source.error or self.session.text or "kart ve USB ağı açık mı, M33 çalışıyor mu?"))
        hello = self.h.connect()
        self.events.put(("hello", hello))
        return "Bağlandı: %s, fw %s" % (hello.get("board"), hello.get("fw"))

    def _do_hello(self):
        self._bg("HELLO…", lambda: self.events.put(("hello", self.h.hello())) or "HELLO tamam")

    def _do_state(self):
        self._bg("STATE…", lambda: self.events.put(("report", self.h.state())) or "STATE tamam")

    def _do_ping(self):
        def go():
            r = [self.h.ping(seq=i + 1) for i in range(4)]
            return "Ping: " + ", ".join("%.1f ms" % x if x is not None else "zaman aşımı" for x in r)
        self._bg("Ping…", go)

    def _do_arp(self):
        def go():
            self.h.board_mac = None
            return "Kart MAC: %s" % eth.mac_str(self.h.resolve())
        self._bg("ARP…", go)

    def _arp_mute(self):
        if self.h:
            self.h.arp_mute = self.arp_mute_var.get()

    def _fcs(self):
        if self.h:
            self.h.fcs_flag = {"none": None, "0": 0, "1": 1}[self.fcs_var.get()]

    def _cmd(self, which):
        def go():
            if which == "START":
                r = self.h.start()
            elif which == "STOP":
                r = self.h.stop()
            else:
                r = self.h.set_rate(self.rate_var.get())
            self.events.put(("resp_show", (which, r)))
            return "%s: %s" % (which, r.status_name)
        self._bg(which + "…", go)

    def _console(self, cmd):
        self._bg(cmd + "…", lambda: "%s -> %s" % (cmd, (self.h.console(cmd) or {}).get("result", "cevap yok")))

    def _clear_injection(self):
        def go():
            self.h.txdelay(0)
            self.h.hold(0)
            return "Enjeksiyon kaldırıldı"
        self._bg("Enjeksiyon kaldırılıyor…", go)

    def _send_cal(self):
        try:
            vals = [int(x) for x in self.cal_var.get().split()]
            if len(vals) != 6:
                raise ValueError
        except ValueError:
            messagebox.showwarning("CAL", "Altı tam sayı girin: ax ay az gx gy gz")
            return
        self._start_cal(vals)

    def calibrate(self):
        """Uçuş ve MP257 sayfalarının düğmesi: CAL 0; kart durunca her şeyi sıfırlar."""
        self._start_cal([0] * 6)

    def _start_cal(self, vals):
        self.source.yaw_zero = 0.0      # sıfırlamayı kart yapar; görüntü ofseti araya girmesin
        self.source.cal_asked = self.h is not None
        self._bg("CAL…", self._cal_job, vals)

    def _cal_job(self, vals):
        """CAL'i kart başlayana (CAL_BUSY) kadar yineler: Host yeniden açılınca kart ilk paketleri eski SEQ diye atar."""
        for _ in range(CAL_TRIES):
            self.h.send_cal(vals)
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                t = self.source.latest
                if t is not None and t.cal_busy:
                    return "CAL kartta başladı: kart durunca 1 s ölçülür, sonra her şey 0"
                time.sleep(0.02)
        return "CAL gönderildi, kart başlamadı (IMU verisi akmıyor ya da |değer| > 4096)"

    def _send_raw(self):
        if not self.h:
            return
        text = self.raw_var.get().strip()
        if text:
            self.h.send_line(text.encode("ascii", "replace") + b"\n")

    def _toggle_gen(self):
        if self.gen_running:
            self.gen_running = False
            self.btn_gen.config(text="Üreteci başlat")
            return
        if not self.h:
            return
        self.gen_running = True
        self.btn_gen.config(text="Üreteci durdur")
        h = self.h

        def loop():
            rng = random.Random(42)
            sent = collections.Counter()
            nxt = {k: time.perf_counter() for k in self.gen_vars}
            while self.gen_running and self.h is h and not h.link.closed:
                now = time.perf_counter()
                for k, var in self.gen_vars.items():
                    try:
                        r = float(var.get())
                    except (tk.TclError, ValueError):
                        r = 0
                    if r <= 0:
                        nxt[k] = now
                        continue
                    if now >= nxt[k]:
                        nxt[k] = max(nxt[k] + 1.0 / r, now - 1.0)
                        try:
                            if k == "env":
                                h.send_env([rng.randint(-3000, 3000) for _ in range(rng.randint(1, 8))])
                            elif k == "bulk":
                                h.send_bulk(rng.getrandbits(32), rng.randint(16, 196))
                            else:
                                h.send_cal([rng.randint(-100, 100) for _ in range(6)])
                            sent[k] += 1
                        except HostError:
                            pass
                self.events.put(("gen", dict(sent)))
                time.sleep(0.005)

        threading.Thread(target=loop, daemon=True).start()

    def _read_counters(self):
        def go():
            c = self.h.get_counters()
            env = self.h.get_env()
            self.events.put(("counters", (c, env)))
            return "GET_STATS okundu (tick %d ms)" % c["_tick"]
        self._bg("GET_STATS…", go)

    def _set_reference(self):
        if self.last_counters is None:
            self.status_var.set("Önce 'Oku' ile sayaçları alın.")
            return
        self.ref_counters = dict(self.last_counters)
        self._show_counters()
        self.status_var.set("Referans alındı (tick %d)." % self.ref_counters["_tick"])

    def _check_equalities(self):
        if self.ref_counters is None or self.last_counters is None:
            self.status_var.set("Önce 'Referans al', sonra trafik, sonra 'Oku'.")
            return
        if self.ref_counters.get("_tick") == self.last_counters.get("_tick"):
            self.status_var.set("Referans ile son okuma aynı an: 'Referans al' → trafik → STOP → 'Oku' → kontrol.")
            return
        d = acc.delta(self.ref_counters, self.last_counters)
        self.eq_tree.delete(*self.eq_tree.get_children())
        bad = 0
        for c in acc.equalities(d):
            bad += 0 if c.ok else 1
            self.eq_tree.insert("", "end", text=("✓ " if c.ok else "✗ ") + c.text, values=(c.layer, c.lhs, c.rhs),
                                tags=("ok" if c.ok else "bad",))
        self.status_var.set("Eşitlikler: %d tutmuyor" % bad if bad else "Eşitliklerin hepsi tutuyor.")

    def _run_tests(self):
        import tests
        ids = [t for t, v in self.test_vars.items() if v.get()]
        if not ids:
            return
        self.test_tree.delete(*self.test_tree.get_children())
        out_dir = os.path.join(HOST_DIR, "logs")

        def go():
            failed = []
            for tid in ids:
                kw = {}
                if tid == "T04":
                    kw["seconds"] = int(self.t04_s.get())
                if tid == "T07":
                    kw["minutes"] = float(self.t07_m.get())
                    kw["slice_s"] = max(10.0, min(300.0, kw["minutes"] * 60.0 / 6.0))
                self.events.put(("test_head", tid))
                result = self._run_one(tests.get(tid), tid, kw)
                if result is not None:
                    path = result.save(out_dir)
                    self.events.put(("test_done", (tid, result.passed, path)))
                    if not result.passed:
                        failed.append(tid)
                else:
                    failed.append(tid)
            return "Testler bitti: %s" % ("hepsi geçti" if not failed else "kalan: " + ", ".join(failed))
        self._bg("Testler çalışıyor…", go)

    def _run_one(self, fn, tid, kw):
        import tests.common as common
        common.ROW_HOOK = lambda row: self.events.put(("test_row", (tid, row)))
        try:
            return fn(self.h, **kw)
        except HostError as e:
            self.events.put(("test_row", (tid, {"girdi": "test durdu", "beklenen": "", "gözlenen": str(e),
                                                "sonuç": "KALDI", "inj": ""})))
            return None
        finally:
            common.ROW_HOOK = None

    # ------------------------------------------------------------ olaylar
    def _chip(self, key, text, color):
        self.chips[key].config(text=text, bg=color)

    def _pump(self):
        n = 0
        try:
            while n < 400:
                kind, data = self.events.get_nowait()
                n += 1
                self._handle(kind, data)
        except queue.Empty:
            pass
        self.root.after(60, self._pump)

    def _handle(self, kind, data):
        if kind == "log":
            self._log(*data)
        elif kind == "report" and data:
            self._on_report(data)
        elif kind == "telem":
            self.plot_a.add(data.accel)
            self.plot_g.add(data.gyro)
        elif kind == "response":
            if data.state != getattr(self, "_last_state", None):     # değişince çiz
                self._last_state = data.state
                self._state_chips(data.state)
        elif kind == "resp_show":
            which, r = data
            extra = " uygulanan %s Hz" % r.applied_rate if r.applied_rate is not None else ""
            self.resp_var.set("Son cevap: %s REQ_ID %d → %s%s | STATE %s | LAT_US %d µs | RTT %.1f ms | deneme %d" % (
                which, r.req_id, r.status_name, extra, proto.state_text(r.state), r.lat_us, r.rtt_ms, r.attempts))
        elif kind == "counters":
            self.last_counters, env = data
            if self.ref_counters is None:
                self.ref_counters = dict(self.last_counters)
            self._show_counters()
        elif kind == "gen":
            self.gen_var.set("gönderilen: " + ", ".join("%s %d" % kv for kv in sorted(data.items())))
        elif kind == "done":
            self.status_var.set(str(data[1]))
        elif kind == "error":
            self.status_var.set("HATA: " + data)
            self._log("rx!", data)
        elif kind == "test_head":
            self.test_tree.insert("", "end", iid="h_" + data, text=data, values=("", "", "…"), tags=("head",), open=True)
            self.test_status.set(data + " çalışıyor…")
        elif kind == "test_row":
            tid, row = data
            parent = "h_" + tid
            if not self.test_tree.exists(parent):
                self.test_tree.insert("", "end", iid=parent, text=tid, tags=("head",), open=True)
            iid = self.test_tree.insert(parent, "end", text=row["girdi"] + (" {%s}" % row["inj"] if row["inj"] else ""),
                                        values=(row["beklenen"], row["gözlenen"], row["sonuç"]), tags=(row["sonuç"],))
            self.test_tree.see(iid)
        elif kind == "test_done":
            tid, ok, path = data
            self.test_tree.item("h_" + tid, values=("", os.path.basename(path) + ".json/.csv",
                                                    "GEÇTİ" if ok else "KALDI"), tags=("head", "GEÇTİ" if ok else "KALDI"))
            self.test_status.set("%s %s" % (tid, "geçti" if ok else "KALDI"))

    def _log(self, direction, text):
        if self.log_pause.get():
            return
        if not self.log_frames.get() and ("FRAME" in text and direction in ("tx", "rx")):
            return
        tag = {"tx": "tx", "rx": "rx", "rx!": "err", "info": "info"}.get(direction, "rx")
        self.log_text.insert("end", "%s %-4s %s\n" % (time.strftime("%H:%M:%S"), direction, text), tag)
        if int(self.log_text.index("end-1c").split(".")[0]) > 3000:
            self.log_text.delete("1.0", "500.0")
        self.log_text.see("end")

    def _state_chips(self, s):
        self._chip("net", "NET_READY", COL_OK if s & proto.STATE_NET_READY else COL_IDLE)
        self._chip("meas", "MEAS_EN", COL_OK if s & proto.STATE_MEAS_EN else COL_IDLE)
        self._chip("fault", "FAULT", COL_BAD if s & proto.STATE_FAULT else COL_IDLE)

    def _on_report(self, rep):
        t = rep.get("type")
        clean = {k: v for k, v in rep.items() if not k.startswith("_")}
        if t not in ("st", "env"):
            self.rep_text.insert("1.0", "%s %s\n" % (time.strftime("%H:%M:%S"), json.dumps(clean, ensure_ascii=False)))
            self.rep_text.delete("40.0", "end")
        if t in ("st", "state"):
            self._state_chips(rep.get("s", 0))
            self._show_resources(rep)
        if t == "fault":
            self._chip("fault", "FAULT " + proto.fault_text(rep.get("why", 0)), COL_BAD if rep.get("set") else COL_IDLE)
        if t == "halt":
            self._chip("fault", "HALT: %s %s" % (rep.get("why"), rep.get("at", "")), COL_BAD)
            messagebox.showerror("Kart durdu", "Kart halt satırı gönderdi:\n%s" % clean)
        if self.h is not None and t in ("hello", "boot", "st", "state"):
            self.info_var.set("Kart: %s | MAC: %s | EPOCH: %s%s | fw: %s" % (
                self.h.last_report.get("hello", {}).get("board", "—"),
                eth.mac_str(self.h.board_mac) if self.h.board_mac else "—", self.h.epoch,
                " (yeniden başladı ×%d)" % self.h.epoch_changes if self.h.epoch_changes else "",
                self.h.last_report.get("hello", {}).get("fw", "—")))

    def _show_resources(self, r):
        tr = self.res_tree
        tr.delete(*tr.get_children())

        def row(name, value, meaning, warn=False):
            text = value if isinstance(value, str) else json.dumps(value)
            tr.insert("", "end", text=name, values=(text, meaning), tags=("warn",) if warn else ())

        inj = r.get("inj", [0, 0, 0])
        row("t / ep", "%s ms / %s" % (r.get("t"), r.get("ep")), "kart tick'i / açılış EPOCH'u")
        row("s / why", "%s / %s" % (proto.state_text(r.get("s", 0)), proto.fault_text(r.get("why", 0))),
            "durum bitleri / FAULT nedenleri", warn=bool(r.get("why")))
        row("inj", inj, "yapay enjeksiyon: maske (1 HOLD, 2 TXDELAY), HOLD ms, TXDELAY ms", warn=bool(inj and inj[0]))
        row("nb", r.get("nb"), "network buffer boş / açılıştan beri en az (18'den)")
        row("ipq", r.get("ipq"), "IP task queue'sunun en az boş yeri")
        row("cp / lp / rp", " / ".join(json.dumps(r.get(k)) for k in ("cp", "lp", "rp")),
            "pool boş / en az: önemli desc (4), önemsiz desc (8), rapor (2)")
        row("q", r.get("q"), "en az boş yer: ctrlQueue (6), workQueue (8), UART TX queue (8)")
        row("sb", r.get("sb"), "en çok doluluk (bayt): sbUartRx (2048), sbCrit (512), sbLow (2048)")
        row("lag", r.get("lag"), "UART RX DMA geride kaldı (veri kaybı) sayısı", warn=bool(r.get("lag")))
        row("heap", r.get("heap"), "heap boş / en az (bayt); ısınmadan sonra düz kalmalı")
        hwm = r.get("hwm", [])
        for name, v in zip(TASK_NAMES, hwm):
            row("hwm " + name, v, "stack'te hiç kullanılmamış en az word (-1 = yok)", warn=(v != -1 and v < 32))
        self.res_age_var.set("Son '%s' raporu: %s" % (r.get("type"), time.strftime("%H:%M:%S")))

    def _show_counters(self):
        if self.last_counters is None:
            return
        tr = self.cnt_tree
        tr.delete(*tr.get_children())
        for i, n in enumerate(COUNTER_NAMES):
            v = self.last_counters.get(n, 0)
            dv = (v - self.ref_counters.get(n, 0)) & 0xFFFFFFFF if self.ref_counters else 0
            if self.only_nonzero.get() and not v and not dv:
                continue
            tr.insert("", "end", values=(i, n, v, dv if dv else ""))

    def _telem_reset(self):
        self.source.reset()
        self.plot_a.clear()
        self.plot_g.clear()

    def _tick(self):
        s = self.source
        t = s.stats()
        direct = t is s.direct_stats
        lat = s.direct if direct else s.udp.whole
        self.tel_vars["rx"].set(str(t.received))
        self.tel_vars["hz"].set("%.1f" % t.rate_now(time.perf_counter()))
        self.tel_vars["gap"].set(str(t.gaps))
        if lat.n:
            self.tel_vars["lat"].set("%.2f / %.2f" % (lat.last / 1000.0, lat.max / 1000.0))
        if t.last is not None:
            self.tel_vars["last"].set(
                "kaynak %s | son: SEQ %d, ivme %s LSB, jiroskop %s LSB, roll %+.2f° pitch %+.2f° yaw %+.2f° | gecikme %s" % (
                    s.active(), t.last.seq, t.last.accel, t.last.gyro, t.last.roll, t.last.pitch, t.last.yaw,
                    "DRDY→UART TX bitti" if direct else "DRDY→Linux'a verildi"))
        self.plot_a.redraw()
        self.plot_g.redraw()
        if self.h is not None:
            self._link_chip()
            lk = self.h.link
            self.linkstat_var.set("hat: gönderilen %d bayt (Linux %d), alınan %d satır (Linux %d) | host sayaçları: %s" % (
                lk.tx_bytes, lk.linux_tx, lk.rx_lines, lk.linux_lines,
                ", ".join("%s %d" % kv for kv in sorted(self.h.c.items()) if "err" in kv[0] or "bad" in kv[0]) or "hata yok"))
        self.root.after(1000, self._tick)

    def _host_event(self, kind, data):
        if kind != "telem":
            self.events.put((kind, data))
        elif not data.linux:                 # Linux'tan gelen kaynaktan zaten geldi
            self.source.on_direct(data)

    def _linux_telem(self, t):
        h = self.h
        if h is not None and h.link.route is not None:
            h.on_linux_telem(t)

    def _linux_line(self, line):
        h = self.h
        if h is not None:
            h.link.feed_line(line)

    def _poll_session(self):
        self.session.poll()
        self.root.after(250, self._poll_session)

    def _close(self):
        self.session.stop()
        self.source.close()
        self.root.destroy()


def main():
    if not FROZEN:
        sys.path.insert(0, HOST_DIR)        # 'tests' paketi; exe'de paketin içinde gelir
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
