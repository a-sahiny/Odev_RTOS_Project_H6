r"""6. madde, 1. aşama kart testi: Linux okurken F746 PC'ye TELEM yollamaz, Linux gidince yollar.

Çalıştırma: python Host\kart_testi_6.py COM6   (TelemetriHost kapalı olmalı: UDP 5002 ve seri port boş)
Kalıcı değişiklik yapmaz; sonunda STOP verir. K4 karttaki telem_udp'yi kill -9 ile öldürür.
"""
import os
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vnet_host import link, source as src, udp_in  # noqa: E402
from vnet_host.hoststack import Host  # noqa: E402

results = []


def check(name, ok, seen):
    results.append(ok)
    print("GECTI" if ok else "KALDI", "|", name, "|", seen)


def rate(read, seconds=1.0):
    n = read()
    time.sleep(seconds)
    return (read() - n) / seconds


def wait(test, limit):
    """test() doğru olana kadar geçen süre, s; olmazsa None."""
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < limit:
        if test():
            return time.perf_counter() - t0
        time.sleep(0.02)
    return None


def silent(read):
    n = read()
    time.sleep(0.3)
    return read() == n


def text(s):
    return "olmadı" if s is None else "%.1f s" % s


def kill9(board=udp_in.BOARD_IP):
    subprocess.run(["ssh", "root@" + board, "kill", "-9", "$(pidof telem_udp)"], check=True, timeout=15)


def main(port="COM6", make_link=lambda p: link.SerialLink(p, 1000000), run=udp_in.run_board, kill=kill9):
    h = Host(make_link(port))
    source = src.TelemSource()
    source.on_line = h.link.feed_line                    # 2. aşamadan sonra cevaplar Linux'tan
    ses = src.BoardSession(source, run=run)
    uart = lambda: h.telem.received                      # noqa: E731
    udp = lambda: source.udp.telem.received              # noqa: E731
    try:
        h.connect()
        ok = h.set_rate(100).ok and h.start().ok
        time.sleep(0.5)
        r = rate(uart)
        check("K1 Linux yokken START: UART'tan TELEM gelir", ok and r > 80, "%.0f Hz" % r)

        error = ses.start()
        t = wait(lambda: source.active() == src.MP257 and silent(uart), 10)
        check("K2 Linux okurken UART TELEM kesilir, UDP akar", not error and t is not None and rate(udp) > 80,
              "kesilme %s, %s" % (text(t), error or "UDP %d paket" % udp()))
        time.sleep(5)
        n = uart()
        time.sleep(5)
        check("K2b 5 s boyunca UART TELEM 0", uart() == n, "%d TELEM" % (uart() - n))
        check("K3 komutlar Linux varken de cevaplanır", h.hello() is not None and h.state() is not None, "HELLO, STATE")

        n = uart()
        ses.stop()
        t = wait(lambda: uart() > n, 5)
        check("K4a Durdur'dan sonra UART TELEM hemen döner (<= 1,5 s)", t is not None and t <= 1.5, text(t))

        ses.start()
        wait(lambda: source.active() == src.MP257 and silent(uart), 10)
        ses.wanted = False                               # yeniden başlatma testi bozmasın
        n = uart()
        kill()
        t = wait(lambda: uart() > n, 8)
        check("K4b telem_udp kill -9: UART TELEM 4 s içinde döner", t is not None and t <= 4.0, text(t))
        ok = h.stop().ok
        time.sleep(0.05)                                 # meas izni örnekten önce okur: STOP anında beklenen örnek de gider
        check("K5 STOP: UART TELEM 50 ms sonra durur", ok and silent(uart), "UART %d TELEM toplam" % uart())
    finally:
        ses.stop()
        source.close()
        h.link.close()
    print("SONUC:", "GECTI" if results and all(results) else "KALDI", "%d/%d" % (sum(results), len(results)))
    return 0 if results and all(results) else 1


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:2]))
