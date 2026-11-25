r"""Gerçek kart testi: App + ssh (telem_udp) + F746 seri port. Kalıcı değişiklik yok; hız sonunda 100 Hz.

Çalıştırma: python Host\kart_testi.py COM6   (TelemetriHost kapalı olmalı: UDP 5002 ve seri port boş)
"""
import os, sys, time, tkinter as tk
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
from vnet_host import gui, link as L, source as src

root = tk.Tk(); app = gui.App(root)
def run(s):
    e = time.perf_counter() + s
    while time.perf_counter() < e:
        root.update(); time.sleep(0.01)
def now(): return app.source.stats().rate_now(time.perf_counter())
res = []
def check(name, ok, obs): res.append(ok); print("GECTI" if ok else "KALDI", "|", name, "|", obs)
try:
    print("baslat:", repr(app.session.start())); app.nb.select(app.flight); run(5)
    check("K-a MP257 akisi", app.source.active() == src.MP257 and abs(now() - 100) < 5, "%.1f Hz, %s" % (now(), app.session.text))
    app._attach(app._make_host(L.SerialLink(PORT, 1000000))); run(4)
    check("K-b HELLO", app.h.last_report.get("hello") is not None, app.info_var.get())
    for hz in (20, 100):
        import threading
        out = {}
        th = threading.Thread(target=lambda: out.update(r=app.h.set_rate(hz))); t0 = time.perf_counter(); th.start()
        hit = None
        while time.perf_counter() - t0 < 4:
            root.update(); time.sleep(0.02)
            if hit is None and abs(now() - hz) <= (1 if hz == 20 else 3): hit = time.perf_counter() - t0
        check("K-c SET_RATE %d" % hz, hit is not None and hit <= 2.0 and out["r"].ok, "%s s sonra %d Hz gorundu, eski ortalama %.1f, cevap %s" % (
            hit and round(hit, 2), hz, app.source.stats().rate_hz(), out["r"].status_name))
    t = threading.Thread(target=app.h.start); t.start(); run(2)
    check("K-d START: dogrudan TELEM gosterilmez", app.source.direct_stats.received == 0,
          "UART'tan gelen %d, gosterilen 0" % app.source.ignored)
    t = threading.Thread(target=app.h.stop); t.start(); run(1.5); n = app.source.udp.telem.received; run(1)
    check("K-e STOP sonrasi Ucus akiyor", app.source.udp.telem.received - n > 80, "1 s'de %d paket" % (app.source.udp.telem.received - n))
    app._disconnect(); n = app.source.udp.telem.received; run(1)
    check("K-f baglanti kesilince akiyor", app.source.udp.telem.received - n > 80, "1 s'de %d paket" % (app.source.udp.telem.received - n))
    u = app.source.udp; app.flight.table.refresh()
    for i in ("f746", "m33", "tail", "whole", "direct"): print("   ", i, u and app.flight.table.tree.item(i, "values"))
    check("K-g toplam = F746 ort + kuyruk ort (+-2)", abs(u.whole.total / u.whole.n - (u.f746.total / u.f746.n + u.tail.total / u.tail.n)) < 2, "gaps %d bozuk %d" % (u.telem.gaps, u.bad))
finally:
    app._close()
print("SONUC:", "GECTI" if all(res) and res else "KALDI", "%d/%d" % (sum(res), len(res)))
