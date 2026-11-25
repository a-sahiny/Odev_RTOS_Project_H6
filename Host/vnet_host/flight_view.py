"""Uçuş (IMU) sekmesi: her TELEM'le gelen roll / pitch / yaw'ı 3B kağıt uçakla gösterir.

Yalnız tkinter Canvas: kendi 3x3 dönme matrisi ve basit perspektif izdüşüm; ek paket yok.

Eksenler kartın IMU eksenleridir (meas_task.c ile aynı): x burun, y sol kanat, z yukarı (masada).
Dönme R = Rz(yaw) · Ry(pitch) · Rx(roll); ivmeölçerin masada gördüğü (−sinθ, sinφ cosθ, cosφ cosθ)
bu matrisle uyumludur (meas_task.c prvAttitudeUpdate, test_meas_att.c).

Renkler Okabe–Ito paletinden (renk körlüğüne dayanıklı); her öğede ayrıca yazı etiketi var,
anlam yalnız renkle verilmez.
"""

import collections
import math
import threading
import time
import tkinter as tk
from tkinter import ttk

from . import proto
from .hoststack import HostError, TelemLatencyStats

# ---------------------------------------------------------------- renk ve yazı tokenları
C = {
    "bg": "#0F1626",            # lacivert zemin
    "grid": "#22304A",
    "grid_far": "#18233A",
    "fg": "#F1F5F9",
    "muted": "#94A3B8",
    "chip_bg": "#141D30",
    "roll": "#F07A4A",          # Okabe–Ito vermilyon (koyu zeminde açıldı)
    "pitch": "#2BC48F",         # Okabe–Ito mavimsi yeşil
    "yaw": "#56B4E9",           # Okabe–Ito gök mavisi
    "axis": "#E2E8F0",
    "plane": (232, 237, 245),   # kağıt
    "warn_bg": "#3B2A0B",
    "warn_fg": "#FCD34D",
    "ok": "#2BC48F",
    "target": "#E69F00",        # 10 ms hedef çizgisi
    "deadline": "#F07A4A",      # 12 ms deadline çizgisi
}
FONT_UI = "Segoe UI"
FONT_NUM = "Consolas"           # rakamlar sabit genişlikte: değer değişirken titremez


def _rot(roll, pitch, yaw):
    """R = Rz(yaw) Ry(pitch) Rx(roll), açılar radyan."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def _mul(m, v):
    return (m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b, k=1.0):
    return (a[0] + k * b[0], a[1] + k * b[1], a[2] + k * b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a):
    n = math.sqrt(_dot(a, a)) or 1.0
    return (a[0] / n, a[1] / n, a[2] / n)


# Kağıt uçak (gövde ekseninde): iki kanat ve aşağı katlanmış iki gövde yüzü.
NOSE = (1.25, 0.0, 0.0)
_PLANE = (
    (NOSE, (-0.80, 0.95, 0.10), (-0.80, 0.10, 0.0)),       # sol kanat (+y)
    (NOSE, (-0.80, -0.95, 0.10), (-0.80, -0.10, 0.0)),     # sağ kanat (−y)
    (NOSE, (-0.80, 0.10, 0.0), (-0.80, 0.0, -0.34)),       # sol gövde katı
    (NOSE, (-0.80, -0.10, 0.0), (-0.80, 0.0, -0.34)),      # sağ gövde katı
)
LIGHT = _norm((0.3, -0.5, 0.8))


class FlightCanvas(tk.Canvas):
    """3B kağıt uçak, gövde eksenleri, dönme yayları, ivme bileşenleri ve açıklama kutusu."""

    CAM_AZ = math.radians(35.0)
    CAM_EL = math.radians(24.0)

    def __init__(self, master, **kw):
        super().__init__(master, background=C["bg"], highlightthickness=0, **kw)
        self.az, self.el = self.CAM_AZ, self.CAM_EL
        self.att = (0.0, 0.0, 0.0)          # derece, görüntülenen (yaw sıfırı uygulanmış)
        self.acc = (0.0, 0.0, 1.0)          # g, gövde ekseninde
        self.bias_ready = True
        self.has_data = False
        self.note = "Veri yok: START verip izlemeyi başlatın."
        self._drag = None
        self.bind("<Configure>", lambda e: self.draw())
        self.bind("<ButtonPress-1>", self._press)
        self.bind("<B1-Motion>", self._move)
        self.bind("<Double-Button-1>", lambda e: self.reset_camera())

    # ------------------------------------------------------------ kamera
    def reset_camera(self):
        self.az, self.el = self.CAM_AZ, self.CAM_EL
        self.draw()

    def _press(self, e):
        self._drag = (e.x, e.y, self.az, self.el)

    def _move(self, e):
        if self._drag is None:
            return
        x0, y0, az0, el0 = self._drag
        self.az = az0 - (e.x - x0) * 0.008
        self.el = max(math.radians(-10), min(math.radians(80), el0 + (e.y - y0) * 0.008))
        self.draw()

    def _camera(self):
        d = 7.0
        c = (d * math.cos(self.el) * math.cos(self.az), d * math.cos(self.el) * math.sin(self.az), d * math.sin(self.el))
        f = _norm((-c[0], -c[1], -c[2]))
        right = _norm(_cross(f, (0.0, 0.0, 1.0)))
        up = _cross(right, f)
        return c, f, right, up

    # ------------------------------------------------------------ veri
    def set_sample(self, roll, pitch, yaw, acc_g, bias_ready):
        self.att = (roll, pitch, yaw)
        self.acc = tuple(acc_g)
        self.bias_ready = bias_ready
        self.has_data = True

    # ------------------------------------------------------------ çizim
    def draw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 100 or h < 100:
            return
        cam, f, right, up = self._camera()
        cx, cy = w * 0.44, h * 0.50
        scale = min(w, h) * 0.26
        self._placed = []           # etiket ve yay kutuları: yeni etiket bunlarla çakışmasın

        def proj(p):
            rel = _sub(p, cam)
            depth = _dot(rel, f)
            k = 7.0 / max(depth, 0.5)
            return (cx + _dot(p, right) * scale * k, cy - _dot(p, up) * scale * k, depth)

        self._ground(proj)
        r, p, y = (math.radians(v) for v in self.att)
        m = _rot(r, p, y)
        prims = []          # (derinlik, çizici)

        # gövde eksenleri (uçakla döner)
        axes = (("x", (1.0, 0, 0), C["axis"], 2.1), ("y", (0, 1.0, 0), C["axis"], 1.7), ("z", (0, 0, 1.0), C["axis"], 1.6))
        for name, v, col, ln in axes:
            tip = _mul(m, (v[0] * ln, v[1] * ln, v[2] * ln))
            tail = _mul(m, (-v[0] * 0.9, -v[1] * 0.9, -v[2] * 0.9))
            for a, b, arrow in ((tail, (0, 0, 0), False), ((0, 0, 0), tip, True)):
                pa, pb = proj(a), proj(b)
                prims.append(((pa[2] + pb[2]) / 2, self._line_fn(pa, pb, col, 2, arrow)))
            pt = proj(_mul(m, (v[0] * (ln + 0.22), v[1] * (ln + 0.22), v[2] * (ln + 0.22))))
            prims.append((pt[2] - 10, self._text_fn(pt, name, C["fg"], (FONT_UI, 15, "bold italic"))))

        # uçak yüzleri (gölgeli)
        for tri in _PLANE:
            wp = [_mul(m, q) for q in tri]
            n = _norm(_cross(_sub(wp[1], wp[0]), _sub(wp[2], wp[0])))
            lum = 0.55 + 0.45 * abs(_dot(n, LIGHT))
            col = "#%02X%02X%02X" % tuple(int(c * lum) for c in C["plane"])
            pp = [proj(q) for q in wp]
            prims.append((sum(q[2] for q in pp) / 3 + 0.05, self._poly_fn(pp, col)))

        # ivme: gövde bileşenleri (g; 1 g = 1,2 birim). a_toplam kullanıcı isteğiyle kalktı (rapor 091).
        if self.has_data:
            k = 1.2
            ax, ay, az = self.acc
            comps = (("a_x", (ax * k, 0, 0), C["roll"]), ("a_y", (0, ay * k, 0), C["pitch"]),
                     ("a_z", (0, 0, az * k), C["yaw"]))
            for lbl, v, col in comps:
                if abs(sum(v)) < 0.02:
                    continue
                tip = _mul(m, v)
                p0, p1 = proj((0, 0, 0)), proj(tip)
                prims.append((p1[2] - 20, self._line_fn(p0, p1, col, 4, True)))
                prims.append((p1[2] - 21, self._chip_fn(proj(_mul(m, _add(v, _norm(v), 0.18))), lbl, col)))

        # dönme yayları ve açı etiketleri
        arcs = (("Roll φ", 0, (1, 0, 0), 2.1, C["roll"], self.att[0]),
                ("Pitch θ", 1, (0, 1, 0), 1.7, C["pitch"], self.att[1]),
                ("Yaw ψ", 2, (0, 0, 1), 1.6, C["yaw"], self.att[2]))
        for name, _, axis, ln, col, val in arcs:
            self._arc(prims, proj, m, axis, ln * 0.62, col, "%s  %+7.2f°" % (name, val))

        # önce geometri (uzaktan yakına), sonra bütün etiketler: etiket hiçbir yüzün altında kalmaz
        for _, fn in sorted(prims, key=lambda t: (getattr(t[1], "is_label", False), -t[0])):
            fn()

        self._legend(w, h)
        self._banner(w)

    def _ground(self, proj):
        z = -1.6
        for i in range(-6, 7):
            a, b = proj((i * 0.6, -3.6, z)), proj((i * 0.6, 3.6, z))
            self.create_line(a[0], a[1], b[0], b[1], fill=C["grid_far"])
            a, b = proj((-3.6, i * 0.6, z)), proj((3.6, i * 0.6, z))
            self.create_line(a[0], a[1], b[0], b[1], fill=C["grid_far"])
        a, b = proj((-3.6, 0, z)), proj((3.6, 0, z))
        self.create_line(a[0], a[1], b[0], b[1], fill=C["grid"])
        a, b = proj((0, -3.6, z)), proj((0, 3.6, z))
        self.create_line(a[0], a[1], b[0], b[1], fill=C["grid"])

    def _arc(self, prims, proj, m, axis, at, col, text):
        """Eksen etrafında sağ el kuralı yönünde ok başlı yay (pozitif dönüş yönü)."""
        u = (axis[1], axis[2], axis[0])               # eksene dik iki birim vektör
        v = _cross(axis, u)
        rad = 0.26
        pts = []
        for i in range(25):
            t = math.radians(30 + 300 * i / 24)
            q = (axis[0] * at + rad * (math.cos(t) * u[0] + math.sin(t) * v[0]),
                 axis[1] * at + rad * (math.cos(t) * u[1] + math.sin(t) * v[1]),
                 axis[2] * at + rad * (math.cos(t) * u[2] + math.sin(t) * v[2]))
            pts.append(proj(_mul(m, q)))
        depth = sum(p[2] for p in pts) / len(pts)
        flat = [c for p in pts for c in p[:2]]
        xs, ys = flat[0::2], flat[1::2]
        self._placed.append((min(xs) - 4, min(ys) - 4, max(xs) + 4, max(ys) + 4))   # etiketler yaydan kaçar

        def fn():
            self.create_line(*flat, fill=col, width=3, smooth=True, arrow="last", arrowshape=(10, 12, 5))
        prims.append((depth - 15, fn))
        lab = proj(_mul(m, (axis[0] * at + rad * 1.9 * u[0], axis[1] * at + rad * 1.9 * u[1],
                            axis[2] * at + rad * 1.9 * u[2])))
        prims.append((depth - 30, self._chip_fn(lab, text, col, big=True)))

    def _line_fn(self, a, b, col, width, arrow):
        def fn():
            self.create_line(a[0], a[1], b[0], b[1], fill=col, width=width,
                             arrow="last" if arrow else "none", arrowshape=(12, 14, 5), capstyle="round")
        return fn

    def _poly_fn(self, pts, col):
        flat = [c for p in pts for c in p[:2]]

        def fn():
            self.create_polygon(*flat, fill=col, outline="#9AA6B8", width=1)
        return fn

    _NUDGE = ((0, 0), (0, -28), (0, 28), (-70, 0), (70, 0), (-70, -28), (70, -28), (-70, 28), (70, 28),
              (0, -56), (0, 56), (-140, 0), (140, 0))

    def _place(self, item, pad):
        """Etiketi, daha önce konan etiketlerle çakışmayan ilk komşu yere kaydırır."""
        x0, y0, x1, y1 = self.bbox(item)
        for dx, dy in self._NUDGE:
            box = (x0 + dx - pad, y0 + dy - pad, x1 + dx + pad, y1 + dy + pad)
            if not any(box[0] < b[2] and b[0] < box[2] and box[1] < b[3] and b[1] < box[3] for b in self._placed):
                self.move(item, dx, dy)
                self._placed.append(box)
                return box
        self._placed.append((x0 - pad, y0 - pad, x1 + pad, y1 + pad))
        return (x0 - pad, y0 - pad, x1 + pad, y1 + pad)

    def _text_fn(self, p, text, col, font):
        def fn():
            t = self.create_text(p[0], p[1], text=text, fill=col, font=font)
            self._place(t, 2)
        fn.is_label = True
        return fn

    def _chip_fn(self, p, text, col, big=False):
        def fn():
            font = (FONT_UI, 11 if big else 10, "bold")
            t = self.create_text(p[0], p[1], text=text, fill=col, font=font)
            x0, y0, x1, y1 = self._place(t, 8)
            r = self.create_rectangle(x0 + 1, y0 + 5, x1 - 1, y1 - 5, fill=C["chip_bg"], outline=col, width=2)
            self.tag_lower(r, t)
        fn.is_label = True
        return fn

    def _legend(self, w, h):
        rows = (("Roll (φ)", C["roll"], "x (burun) ekseni etrafında dönüş · a_x"),
                ("Pitch (θ)", C["pitch"], "y (kanat) ekseni etrafında dönüş · a_y"),
                ("Yaw (ψ)", C["yaw"], "z (üst) ekseni etrafında dönüş · a_z"))
        lw, lh = 372, 22 * len(rows) + 14
        x0, y0 = w - lw - 14, h - lh - 14
        self.create_rectangle(x0, y0, x0 + lw, y0 + lh, fill="#111A2C", outline="#3B4A66", width=1)
        for i, (name, col, desc) in enumerate(rows):
            y = y0 + 18 + 22 * i
            self.create_rectangle(x0 + 12, y - 7, x0 + 18, y + 7, fill=col, outline=col)
            self.create_text(x0 + 28, y, text=name, anchor="w", fill=C["fg"], font=(FONT_UI, 10, "bold"))
            self.create_text(x0 + 108, y, text=": " + desc, anchor="w", fill=C["muted"], font=(FONT_UI, 9))

    def _banner(self, w):
        if not self.has_data:
            text, bg, fg = self.note, "#1E293B", C["muted"]
        elif not self.bias_ready:
            text, bg, fg = "Jiroskop bias'ı ölçülüyor (START'tan sonraki 1 s): kartı oynatmayın.", C["warn_bg"], C["warn_fg"]
        else:
            text, bg, fg = ("Yaw yalnız jiroskopla hesaplanır (manyetometre yok): zamanla kayar. "
                            "Sürükle: kamera · çift tık: kamerayı sıfırla"), "#13203A", C["muted"]
        t = self.create_text(16, 16, text=text, anchor="nw", fill=fg, font=(FONT_UI, 10))
        x0, y0, x1, y1 = self.bbox(t)
        r = self.create_rectangle(x0 - 8, y0 - 5, x1 + 8, y1 + 5, fill=bg, outline="")
        self.tag_lower(r, t)


class DeadlineHistogram(tk.Canvas):
    """Kartın ölçtüğü TELEM gecikmesi (DRDY -> TX bitti) dağılımı: 0–20 ms, 1 ms kutular; 10 ms hedef ve
    12 ms deadline çizgileri etiketli. Windows / USB yok."""

    MAX_MS = 20

    def __init__(self, master, **kw):
        super().__init__(master, background="white", highlightthickness=1, highlightbackground="#bbb", **kw)
        self.values = []
        self.bind("<Configure>", lambda e: self.draw())

    def set_values(self, values):
        self.values = list(values)
        self.draw()

    def draw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 80 or h < 60:
            return
        left, right, top, bottom = 34, w - 8, 20, h - 20
        bins = [0] * (self.MAX_MS + 1)
        for v in self.values:
            bins[min(self.MAX_MS, int(v))] += 1
        peak = max(bins) or 1
        bw = (right - left) / len(bins)
        self.create_text(left, 4, anchor="nw", text="Kart DRDY → TX bitti dağılımı (son %d TELEM)" % len(self.values),
                         font=(FONT_UI, 9, "bold"))
        for i, n in enumerate(bins):
            x0 = left + i * bw
            y0 = bottom - (bottom - top) * n / peak
            col = "#1976d2" if i < 10 else ("#ef6c00" if i < 12 else "#c62828")
            if n:
                self.create_rectangle(x0 + 1, y0, x0 + bw - 1, bottom, fill=col, outline="")
        for ms in range(0, self.MAX_MS + 1, 2):
            x = left + ms * bw
            self.create_text(x, bottom + 3, anchor="n", text=str(ms) if ms < self.MAX_MS else "≥20",
                             font=(FONT_UI, 8), fill="#555")
        for ms, col, txt, anchor, dx in ((10, C["target"], "hedef 10 ms", "ne", -3),
                                         (12, C["deadline"], "deadline 12 ms", "nw", 3)):
            x = left + ms * bw
            self.create_line(x, top - 2, x, bottom, fill=col, width=2, dash=(5, 3))
            self.create_text(x + dx, top - 2, anchor=anchor, text=txt, fill=col, font=(FONT_UI, 8, "bold"))
        self.create_line(left, bottom, right, bottom, fill="#999")
        self.create_text(left - 4, top, anchor="ne", text=str(peak), font=(FONT_UI, 8), fill="#555")
        self.create_text(right, bottom - 2, anchor="se", text="ms", font=(FONT_UI, 8), fill="#555")


class FlightTab(ttk.Frame):
    """Sekme: sol uçuş görünümü, sağ sayısal panel + gecikme paneli + düğmeler. Veri TELEM'den gelir
    (her IMU örneğinde bir tane); host hiçbir şey sormaz."""

    def __init__(self, master, app):
        super().__init__(master, padding=8)
        self.app = app                      # gui.App: .h (Host) ve .status_var
        self.watching = None                # izlenen Host (dinleyici bağlıyken)
        self.stats = TelemLatencyStats()
        self.latest = None                  # son Telem (host'un alma thread'i yazar)
        self.drawn = None
        self.yaw_zero = 0.0
        self.board0 = None                  # izleme başında kart sayaçları (fark için)
        self.board_text = "Kart sayaçları (izleme başı → sonu): —"
        self.columnconfigure(0, weight=3)
        self.columnconfigure(1, weight=1, minsize=330)
        self.rowconfigure(0, weight=1)

        self.view = FlightCanvas(self, width=760, height=560)
        self.view.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        side = ttk.Frame(self)
        side.grid(row=0, column=1, sticky="nsew")

        g = ttk.LabelFrame(side, text="TELEM (UDP 5002, her IMU örneği)", padding=8)
        g.pack(fill="x")
        g.columnconfigure((0, 1), weight=1)
        self.btn_start = ttk.Button(g, text="▶ START ve izle", command=self.start)
        self.btn_start.grid(row=0, column=0, sticky="ew", padx=2, pady=2)
        self.btn_stop = ttk.Button(g, text="■ İzlemeyi durdur", command=self.stop, state="disabled")
        self.btn_stop.grid(row=0, column=1, sticky="ew", padx=2, pady=2)
        ttk.Label(g, text="IMU hızı (Hz), START'tan önce").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.rate_var = tk.IntVar(value=proto.RATE_DEFAULT_HZ)
        ttk.Combobox(g, textvariable=self.rate_var, values=proto.RATES_HZ, state="readonly", width=6).grid(
            row=1, column=1, sticky="w", pady=(6, 0))
        ttk.Button(g, text="Yaw sıfırla (görüntü)", command=self.zero_yaw).grid(row=2, column=0, sticky="ew", padx=2, pady=2)
        ttk.Button(g, text="Kamerayı sıfırla", command=self.view.reset_camera).grid(row=2, column=1, sticky="ew", padx=2,
                                                                                   pady=2)
        ttk.Label(g, text="Kart START ile STOP arasında her IMU örneğini (DRDY) TELEM olarak yollar.",
                  style="Small.TLabel", wraplength=300).grid(row=3, column=0, columnspan=2, sticky="w", pady=(4, 0))

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

        g = ttk.LabelFrame(side, text="Örnek", padding=8)
        g.pack(fill="x", pady=(8, 0))
        self.sample_var = tk.StringVar(value="—")
        ttk.Label(g, textvariable=self.sample_var, font=(FONT_NUM, 9), justify="left").pack(anchor="w")

        from .udp_view import LatencyPanel            # udp_view bu modülü içe aktarır: döngü olmasın diye burada
        LatencyPanel(side, lambda: getattr(getattr(app, "udp", None), "rx", None)).pack(fill="x", pady=(8, 0))

        g = ttk.LabelFrame(side, text="Gecikme: hedef 10 ms, deadline 12 ms", padding=8)
        g.pack(fill="both", expand=True, pady=(8, 0))
        self.hist = DeadlineHistogram(g, height=130)
        self.hist.pack(fill="x")
        self.dl_var = tk.StringVar(value="—")
        ttk.Label(g, textvariable=self.dl_var, font=(FONT_NUM, 9), justify="left").pack(anchor="w", pady=(6, 0))
        self.board_var = tk.StringVar(value=self.board_text)
        ttk.Label(g, textvariable=self.board_var, font=(FONT_UI, 9, "bold"), wraplength=310,
                  justify="left").pack(anchor="w", pady=(6, 0))
        ttk.Label(g, text="Yalnız kart zamanı: IMU verisi hazır (DRDY kesmesi) → TELEM satırının son biti "
                          "UART'tan çıktı (TX bitti kesmesi). PC / USB yok. Bir TELEM'in süresi o gittikten "
                          "sonra bilinir, bu yüzden bir sonraki TELEM'de gelir. Durdurunca panel sıfırlanır.",
                  style="Small.TLabel", wraplength=310, justify="left").pack(anchor="w", pady=(4, 0))

        self.after(33, self._frame)
        self.after(500, self._panel)

    # ------------------------------------------------------------ düğmeler
    def _host(self):
        return self.app.h

    def start(self):
        h = self._host()
        if h is None:
            self.app.status_var.set("Önce karta ya da simülatöre bağlanın.")
            return
        self.btn_start.config(state="disabled")
        hz = int(self.rate_var.get() or proto.RATE_DEFAULT_HZ)
        self._reset_panel()

        def go():
            try:
                c = h.get_counters()
                self.board0 = (c["telem_over10"], c["telem_miss"])
                r = h.set_rate(hz)
                if not r.ok:
                    raise HostError("SET_RATE %d reddedildi: %s" % (hz, r.status_name))
                self.stats.reset()
                self.watching = h
                h.listeners.append(self._on_event)
                r = h.start()
                if not r.ok:
                    h.listeners.remove(self._on_event)
                    self.watching = None
                    self.app.events.put(("error", "START reddedildi: %s" % r.status_name))
                    self.after(0, lambda: self.btn_start.config(state="normal"))
                    return
                self.app.events.put(("done", ("", "TELEM izleniyor (%d Hz)" % hz)))
                self.after(0, lambda: self.btn_stop.config(state="normal"))
            except HostError as e:
                self.app.events.put(("error", str(e)))
                self.after(0, lambda: self.btn_start.config(state="normal"))
        threading.Thread(target=go, daemon=True).start()

    def stop(self):
        """İzleme durur; gecikme paneli sıfırlanır (bir sonraki izleme baştan başlar). Kartın bu
        izleme boyunca saydığı aşımlar durum çubuğuna yazılır."""
        h, w, b0 = self._host(), self.watching, self.board0
        if w is not None:
            try:
                w.listeners.remove(self._on_event)
            except ValueError:
                pass
        self.watching = None
        self.board0 = None
        s = self.stats.summary()
        self._reset_panel()
        self.btn_stop.config(state="disabled")
        self.btn_start.config(state="normal")
        self.app.status_var.set("İzleme durdu, gecikme paneli sıfırlandı (ölçüm sürüyor; STOP için Kontrol sekmesi)")
        if w is not None and h is w and b0 is not None:
            def go():
                try:
                    c = h.get_counters()
                    self.app.events.put(("done", ("", "Son izleme: kart DRDY→TX >10 ms %d, >12 ms %d (sayaç farkı); "
                                                  "TELEM'lerle gelen %d ölçümde en çok %.2f ms" % (
                                                      c["telem_over10"] - b0[0], c["telem_miss"] - b0[1],
                                                      s["n"], s["max_ms"]))))
                except Exception:
                    pass
            threading.Thread(target=go, daemon=True).start()

    def _reset_panel(self):
        self.stats.reset()
        self.hist.set_values([])
        self.dl_var.set("—")
        self.board_var.set(self.board_text)

    def detach(self):
        """Bağlantı kesilince."""
        self.stop()
        self.latest = None

    def zero_yaw(self):
        if self.latest is not None:
            self.yaw_zero = self.latest.yaw
        self.drawn = None

    def _on_event(self, kind, data):
        if kind == "telem":
            self.stats.add(data)
            self.latest = data              # tek atama: çizim döngüsü en sonuncuyu okur

    # ------------------------------------------------------------ çizim döngüleri
    def _frame(self):
        t = self.latest
        if t is not None and t is not self.drawn and self.winfo_ismapped():
            yaw = ((t.yaw - self.yaw_zero + 180.0) % 360.0) - 180.0
            self.view.set_sample(t.roll, t.pitch, yaw, t.accel_g, t.bias_ready)
            self.view.draw()
            self.drawn = t
            self.ang_vars["roll"].set("%+.2f°" % t.roll)
            self.ang_vars["pitch"].set("%+.2f°" % t.pitch)
            self.ang_vars["yaw"].set("%+.2f°" % yaw)
            a, g = t.accel_g, t.gyro_dps
            self.sample_var.set(
                "SEQ %d\n"
                "ivme  (g)    x %+6.3f  y %+6.3f  z %+6.3f\n"
                "jiro  (°/s)  x %+7.1f  y %+7.1f  z %+7.1f\n"
                "bias %s · ivme düzeltmesi %s · önceki TELEM %s" % (
                    t.seq, a[0], a[1], a[2], g[0], g[1], g[2],
                    "hazır" if t.bias_ready else "ölçülüyor", "var" if t.acc_used else "yok",
                    "SEQ %d: %.2f ms" % (t.prev_seq, t.prev_ms) if t.prev_valid else "yok"))
        elif self.drawn is None and t is None and self.winfo_ismapped():
            self.view.draw()
            self.drawn = False
        self.after(33, self._frame)

    def _panel(self):
        h = self.watching
        if h is not None and self.winfo_ismapped():
            s = self.stats
            self.hist.set_values(list(s.values)[-500:])
            n = max(s.n, 1)
            last = s.values[-1] if s.values else 0.0
            self.dl_var.set(
                "TELEM %d  %.1f Hz  kayıp (SEQ boşluğu) %d\n"
                "DRDY→TX  son %.2f  ort %.2f  p99 %.2f  en çok %.2f ms\n"
                "≤10 ms %5.1f%%   ≤12 ms %5.1f%%   >12 ms %d\n"
                "ölçüm %d  (bir öncekinden eski: %d)" % (
                    h.telem.received, h.telem.rate_hz(), h.telem.gaps,
                    last, s.mean(), s.percentile(0.99), s.max,
                    100.0 * s.le_target / n, 100.0 * s.le_deadline / n, s.over_deadline, s.n, s.behind))
            self.board_var.set("Kart DRDY → TX bitti (%d ölçüm): %s" % (
                s.n, "✓ deadline tuttu (hepsi ≤ 12 ms)" if s.over_deadline == 0 else
                "✗ %d TELEM 12 ms'yi aştı" % s.over_deadline))
        self.after(500, self._panel)
