r"""6. madde, 4. aşama kart testi: arayüzün Otomatik bağlantısı, seri portlu ve seri portsuz.

Çalıştırma: python Host\kart_testi_a4.py   (TelemetriHost kapalı: UDP 5002, 5003 ve COM boş). Kalıcı değişiklik yapmaz.
"""
import os
import sys
import threading
import time
import tkinter as tk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vnet_host import gui, link, source as src  # noqa: E402
from kart_testi_6 import check, results  # noqa: E402


def run_app(root, seconds):
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        root.update()
        time.sleep(0.01)


def call(root, fn):
    """fn'i arka planda çalıştırır, arayüz dönerken sonucu bekler."""
    out = []

    def work():
        try:
            out.append(fn())
        except Exception as e:
            out.append(e)
    t = threading.Thread(target=work)
    t.start()
    while t.is_alive():
        root.update()
        time.sleep(0.01)
    return out[0]


def one(name, port):
    root = tk.Tk()
    app = gui.App(root)
    real = gui.linkmod.guess_stlink_port
    gui.linkmod.guess_stlink_port = lambda: port
    try:
        app._connect_serial()
        end = time.perf_counter() + 15
        while app.busy and time.perf_counter() < end:
            run_app(root, 0.1)
        run_app(root, 0.5)
        lk = app.h.link
        check("%s: bağlanır" % name, app.status_var.get().startswith("Bağlandı"), app.status_var.get())
        check("%s: hat türü" % name, isinstance(lk, link.SerialLink if port else link.LinuxLink), type(lk).__name__)
        rates = [call(root, lambda hz=hz: app.h.set_rate(hz)) for hz in (20, 100)]
        c = call(root, app.h.get_counters)
        check("%s: SET_RATE, GET_STATS" % name, all(getattr(r, "ok", False) for r in rates) and isinstance(c, dict), str(rates[0])[:60])
        run_app(root, 2)
        check("%s: Uçuş akıyor, durum kutusu" % name, app.source.active() == src.MP257 and
              app.chips["link"].cget("text") == "BAĞLI: MP257 (Linux)", "%s, %.0f Hz" % (
                  app.chips["link"].cget("text"), app.source.stats().rate_now(time.perf_counter())))
        if not port:
            check("%s: bütün trafik Linux'tan" % name, lk.tx_bytes == lk.linux_tx > 0 and lk.rx_lines == lk.linux_lines > 0
                  and lk.dropped == 0, "tx %d / Linux %d, rx %d / Linux %d" % (lk.tx_bytes, lk.linux_tx, lk.rx_lines, lk.linux_lines))
        app._disconnect()
    finally:
        gui.linkmod.guess_stlink_port = real
        app._close()
        time.sleep(1.5)


if __name__ == "__main__":
    f746 = link.guess_stlink_port()
    check("F746 portu bulundu", f746 is not None, "%s; portlar %s" % (f746, [p[0] for p in link.list_ports()]))
    one("seri portsuz", None)
    if f746:
        one("seri portlu", f746)
    print("SONUC:", "GECTI" if all(results) else "KALDI", "%d/%d" % (sum(results), len(results)))
    sys.exit(0 if all(results) else 1)
