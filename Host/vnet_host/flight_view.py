"""Her TELEM'le gelen roll / pitch / yaw'ı 3B kağıt uçakla çizen görünüm (imu_page.py kullanır).

Yalnız tkinter Canvas: kendi 3x3 dönme matrisi ve basit perspektif izdüşüm; ek paket yok.

Eksenler kartın IMU eksenleridir (meas_task.c ile aynı): x burun, y sol kanat, z yukarı (masada).
Dönme R = Rz(yaw) · Ry(pitch) · Rx(roll); ivmeölçerin masada gördüğü (−sinθ, sinφ cosθ, cosφ cosθ)
bu matrisle uyumludur (meas_task.c prvAttitudeUpdate, test_meas_att.c).

Renkler Okabe–Ito paletinden (renk körlüğüne dayanıklı); her öğede ayrıca yazı etiketi var,
anlam yalnız renkle verilmez.
"""

import math
import tkinter as tk

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
        if h < 440:                     # alçak görünümde açı etiketlerini örter
            return
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
