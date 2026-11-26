"""Uçuş simülatörü: karttaki süzgeci (App/Src/attitude.c, gcc ile derlenir) eski süzgeçle karşılaştırır.

Kullanım: python IMU/test/ucus_sim.py            tabloyu basar
          python IMU/test/ucus_sim.py --json D   grafik verisini D dosyasına da yazar
Gerekenler: gcc, numpy, scipy. Kart gerekmez.

Model: dört motorlu, itkisi gövde z'sinde, yatay sürüklemesi 0,4 1/s; 3 s yerde, sonra rastgele (tohumlu)
roll / pitch / yaw. Sensör: bias, ölçek hatası, gürültü, kırpma, 1. derece DLPF + 2 ms, sayısallaştırma.
"""
import ctypes as ct
import json
import math
import os
import subprocess
import sys
import tempfile

import numpy as np
from scipy.signal import butter, lfilter, sosfiltfilt

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
D = math.pi / 180
FS = 8000                                   # gerçek hareketin hesap hızı
G0 = 9.80665
OUT = np.dtype([("acc", "<f4", 3), ("gyro", "<f4", 3), ("angle", "<f4", 3), ("flags", "u1"), ("pad", "u1", 3)])
WRAPPER = """#include "attitude.h"
void run_all(const AttPoint *pt, size_t blocks, size_t n, float dt, AttOutput *out)
{ for (size_t k = 0; k < blocks; ++k) { att_update(pt + k * n, n, dt, out + k); } }
"""
SCENES = (("sakin", dict(tilt=8, agile=0.4, vib_g=1.5, vib_dps=15), {}),
          ("çevik", dict(tilt=16, agile=0.8, vib_g=2.5, vib_dps=30), {}),
          ("sert titreşim", dict(tilt=12, agile=0.5, vib_g=5.0, vib_dps=60), {}),
          ("uzun hızlanma", dict(tilt=18, agile=0.12, vib_g=2.0, vib_dps=20), {}),
          ("motor ısısı", dict(tilt=14, agile=0.4, vib_g=2.5, vib_dps=25), dict(sc=0.02, warm=(0.6, -0.5, 0.4))))


def build():
    tmp = tempfile.mkdtemp(prefix="att_")
    w = os.path.join(tmp, "sarici.c")
    open(w, "w").write(WRAPPER)
    dll = os.path.join(tmp, "attitude.dll")
    subprocess.run(["gcc", "-std=gnu11", "-O2", "-Wall", "-Wextra", "-Werror", "-shared", "-I" + os.path.join(ROOT, "App", "Inc"),
                    os.path.join(ROOT, "App", "Src", "attitude.c"), w, "-o", dll, "-lm"], check=True)
    d = ct.CDLL(dll)
    d.att_set_rate.argtypes = [ct.c_float, ct.c_float]
    d.run_all.argtypes = [ct.c_void_p, ct.c_size_t, ct.c_size_t, ct.c_float, ct.c_void_p]
    return d


# ---------------------------------------------------------------- gerçek hareket
def smooth(rng, n, fc, amp):
    """Bant sınırlı rastgele hareket (100 Hz'te üretilir, FS'e çekilir)."""
    y = sosfiltfilt(butter(2, fc / 50.0, output="sos"), rng.standard_normal(n // 80 + 400))[200:200 + n // 80 + 2]
    return np.interp(np.arange(n) / 80.0, np.arange(len(y)), y / np.std(y) * amp)


def quat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r / 2), np.sin(r / 2), np.cos(p / 2), np.sin(p / 2), np.cos(y / 2), np.sin(y / 2)
    return np.stack([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def body_rate(q):
    a, b, c, d = np.gradient(q, 1.0 / FS, axis=1)
    w, x, y, z = q
    return 2 * np.stack([w * b - x * a - y * d + z * c, w * c + x * d - y * a - z * b, w * d - x * c + y * b - z * a])


def flight(sec, seed, tilt, agile, vib_g, vib_dps, yaw_amp=40.0, rest=3.0):
    """Döner: q, gerçek açısal hız (rad/s), titreşimsiz özgül kuvvet (g), titreşim ivmesi, titreşim hızı."""
    rng = np.random.default_rng(seed)
    n = int(sec * FS)
    env = np.clip((np.arange(n) / FS - rest) / 1.0, 0, 1)
    env = env * env * (3 - 2 * env)
    q = quat(env * smooth(rng, n, agile, tilt * D), env * smooth(rng, n, agile, tilt * D), env * smooth(rng, n, 0.15, yaw_amp * D))
    w = body_rate(q)
    zx, zy = 2 * (q[1] * q[3] + q[0] * q[2]), 2 * (q[2] * q[3] - q[0] * q[1])
    zz = 1 - 2 * (q[1] ** 2 + q[2] ** 2)
    k = 0.4
    al = math.exp(-k / FS)
    horiz = [f * G0 - k * lfilter([(1 - al) / k], [1, -al], f * G0) for f in (zx / zz, zy / zz)]
    aw = np.stack([horiz[0] / G0, horiz[1] / G0, np.ones(n)])
    rot = np.array([[1 - 2 * (q[2] ** 2 + q[3] ** 2), 2 * (q[1] * q[2] - q[0] * q[3]), zx],
                    [2 * (q[1] * q[2] + q[0] * q[3]), 1 - 2 * (q[1] ** 2 + q[3] ** 2), zy],
                    [2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[2] * q[3] + q[0] * q[1]), zz]])
    sf = np.einsum("jin,jn->in", rot, aw)
    va, vw = np.zeros((3, n)), np.zeros((3, n))
    for _ in range(4):                      # dört motor: 110-190 Hz dolaşan devir ve 2. harmoniği
        f = 150 + 25 * smooth(rng, n, 0.3, 0.6) + rng.uniform(-12, 12)
        ph = 2 * np.pi * np.cumsum(f) / FS + rng.uniform(0, 6)
        for h, g in ((1, 1.0), (2, 0.35)):
            for i in range(3):
                o = rng.uniform(0, 2 * np.pi)
                va[i] += g * vib_g / 4 * (1.0 if i == 2 else 0.6) * np.sin(h * ph + o)
                vw[i] += g * vib_dps / 4 * D * (0.5 if i == 2 else 1.0) * np.sin(h * ph + o + 1.0)
    return q, w, sf, va * env, vw * env


def sensor(w, sf, seed, dlpf_g, dlpf_a, acc_fs, gyro_fs, hz, sc=0.007, warm=(0, 0, 0)):
    rng = np.random.default_rng(seed + 1000)
    n = w.shape[1]
    bias, abias = (1.168, 2.698, 2.281), (-.005, .003, -.009)
    ramp = np.clip((np.arange(n) / FS - 10) / 60, 0, 1)

    def dlpf(x, fc):
        al = 1 - math.exp(-2 * math.pi * fc / FS)
        y = lfilter([al], [1, -(1 - al)], x, zi=[x[0] * (1 - al)])[0]
        return np.concatenate([np.full(16, y[0]), y[:-16]])
    step = FS // hz
    g, a = [], []
    for i in range(3):
        x = w[i] / D * (1 + sc * (1 if i != 1 else -1)) + bias[i] + ramp * warm[i] + rng.standard_normal(n) * 0.63
        g.append(dlpf(np.clip(x + np.cumsum(rng.standard_normal(n)) * 2e-5, -gyro_fs, gyro_fs), dlpf_g)[step - 1::step])
        a.append(dlpf(np.clip(sf[i] + abias[i] + rng.standard_normal(n) * 0.019, -acc_fs, acc_fs), dlpf_a)[step - 1::step])
    kg, ka = 32768 / gyro_fs, 32768 / acc_fs
    return np.round(np.stack(a) * ka) / ka, np.round(np.stack(g) * kg) / kg


# ---------------------------------------------------------------- süzgeçler
def run_new(d, a, g, hz, n):
    d.att_init()
    d.att_set_rate(hz / n, hz)
    d.att_restart()
    m = a.shape[1] // n
    pts = np.concatenate([a, g * D]).T[:m * n].astype(np.float32).copy(order="C")
    out = np.zeros(m, OUT)
    d.run_all(pts.ctypes.data, m, n, 1.0 / hz, out.ctypes.data)
    return out


def run_old(a, g, hz):
    """Eski meas_task.c: Euler tamamlayıcı süzgeç, zaman sabiti 0,49 s, ivme kapısı 0,85-1,15 g."""
    m, dt = a.shape[1], 1.0 / hz
    al = 0.49 / (0.49 + dt)
    ang = np.zeros((m, 3))
    r = p = y = 0.0
    b = g[:, :hz].mean(axis=1)
    for k in range(m):
        ax, ay, az = a[:, k]
        ok = 0.85 <= math.sqrt(ax * ax + ay * ay + az * az) <= 1.15
        ar, ap = math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az))
        if k < hz:
            if ok or k == 0:
                r, p = ar, ap
        else:
            wx, wy, wz = (g[:, k] - b) * D
            sr, cr, cp = math.sin(r), math.cos(r), math.cos(p)
            qr = wy * sr + wz * cr
            if abs(p) < 85 * D:
                r, y = r + (wx + qr * math.sin(p) / cp) * dt, y + qr / cp * dt
            else:
                r += wx * dt
            p += (wy * cr - wz * sr) * dt
            if ok:
                r += (1 - al) * ((ar - r + math.pi) % (2 * math.pi) - math.pi)
                p += (1 - al) * (ap - p)
        ang[k] = r, p, y
    return ang


def truth(q, step):
    w, x, y, z = q[:, step - 1::step]
    return np.stack([np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)), np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)),
                     np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))], axis=1)


def tilt_error(ang, tr):
    e = np.degrees((ang - tr + math.pi) % (2 * math.pi) - math.pi)[5 * 200:]
    t = np.hypot(e[:, 0], e[:, 1])
    return math.sqrt((t ** 2).mean()), t.max()


def rate_error(y, w):
    """Çıkış hızının artığı (°/s), en iyi gecikmede; gecikme ms."""
    tr = w[:, 7::8] / D
    best = min((math.sqrt((((y - np.roll(tr, k, axis=1)[:, 4::5][:, :y.shape[1]])[:, 1000:]) ** 2).mean()), k) for k in range(16))
    return best


def main():
    d = build()
    data = {}
    print("%-15s %22s %22s %20s" % ("", "eski: RMS / en çok °", "yeni: RMS / en çok °", "jiro artığı °/s"))
    for name, kw, sk in SCENES:
        rows = []
        for seed in (1, 2, 3):
            q, w, sf, va, vw = flight(90, seed, **kw)
            tr = truth(q, FS // 200)
            a, g = sensor(w + vw, sf + va, seed, 92, 99, 4.0, 1000.0, 200, **sk)
            old = run_old(a, g, 200)
            a2, g2 = sensor(w + vw, sf + va, seed, 184, 218, 16.0, 2000.0, 1000, **sk)
            new = run_new(d, a2, g2, 1000, 5)
            bo = g - np.array([[1.168], [2.698], [2.281]])
            ro, rn = rate_error(bo, w), rate_error(np.degrees(new["gyro"]).T, w)
            rows.append(tilt_error(old, tr) + tilt_error(new["angle"], tr) + ro + rn)
            if seed == 1:
                s = slice(0, 60 * 200, 20)
                data[name] = {"tr": np.degrees(tr[s, 0]).round(2).tolist(), "old": np.degrees(old[s, 0]).round(2).tolist(),
                              "new": np.degrees(new["angle"][s, 0]).round(2).tolist()}
        m = np.mean(rows, axis=0)
        data[name]["sum"] = m.round(3).tolist()
        print("%-15s %12.2f / %6.1f %13.2f / %6.1f %9.2f -> %.2f  (gecikme %d -> %d ms)" % (
            name, m[0], m[1], m[2], m[3], m[4], m[6], m[5], m[7]), flush=True)
    if len(sys.argv) > 2 and sys.argv[1] == "--json":
        json.dump(data, open(sys.argv[2], "w"), separators=(",", ":"))


if __name__ == "__main__":
    main()
