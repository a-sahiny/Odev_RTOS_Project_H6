r"""6. madde, 2. aşama kart testi: Linux okurken F746'nın satırları (cevap, rapor) Linux üzerinden gelir.

Çalıştırma: python Host\kart_testi_a2.py COM6 [dakika]   (TelemetriHost kapalı: UDP 5002, 5003 ve seri port boş)
D6 [dakika] sürer (varsayılan 5). Kalıcı değişiklik yapmaz; D7 karttaki telem_udp'yi kill -9 ile öldürür.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vnet_host import link, source as src  # noqa: E402
from vnet_host.hoststack import Host  # noqa: E402
from kart_testi_6 import check, kill9, results, text, wait  # noqa: E402


def main(port="COM6", minutes="5"):
    h = Host(link.SerialLink(port, 1000000))
    lk = h.link
    source = src.TelemSource()
    source.on_line = lk.feed_line
    ses = src.BoardSession(source)
    uart = lambda: lk.rx_lines - lk.linux_lines          # noqa: E731

    def quiet(seconds):
        n = uart()
        time.sleep(seconds)
        return uart() - n

    try:
        h.connect()
        ok = h.set_rate(100).ok and h.stop().ok
        n = uart()
        time.sleep(3)
        check("D5a Linux yokken satırlar UART'tan", ok and uart() > n and lk.linux_lines == 0, "3 s'de %d satır" % (uart() - n))

        error = ses.start()
        t = wait(lambda: source.active() == src.MP257 and lk.linux_lines > 0 and quiet(1.5) == 0, 12)
        n, m = uart(), lk.linux_lines
        time.sleep(5)
        check("D5b Linux okurken UART'tan satır 0, 'st' raporları Linux'tan", not error and t is not None and uart() == n
              and lk.linux_lines - m >= 4, "5 s'de UART %d, Linux %d satır %s" % (uart() - n, lk.linux_lines - m, error))

        m = lk.linux_lines
        hello, state, c = h.hello(), h.state(), h.get_counters()
        check("D5c HELLO, STATE, GET_STATS cevabı Linux'tan", bool(hello and state and c) and uart() == n,
              "Linux %d satır, UART %d" % (lk.linux_lines - m, uart() - n))

        rtts, fails = [], 0
        for _ in range(100):
            try:
                rtts.append(h.command(0x04, b"\x03", timeout=0.5, retries=0).rtt_ms)
            except Exception:
                fails += 1
        check("D5d 100 GET_STATS, tekrar denemesiz", fails == 0, "cevapsız %d, RTT ort %.1f en çok %.1f ms" % (
            fails, sum(rtts) / max(1, len(rtts)), max(rtts or [0])))

        source.udp.reset()
        lost0, err0 = source.lines.join.lost, dict(h.c)
        end = time.perf_counter() + 60 * float(minutes)
        cmds = fails = 0
        while time.perf_counter() < end:                 # yük: saniyede 10 GET_STATS
            try:
                h.command(0x04, b"\x03", timeout=0.5, retries=0)
            except Exception:
                fails += 1
            cmds += 1
            time.sleep(0.1)
        u = source.udp
        check("D6a %s dk, 100 Hz, yükle: TELEM kaybı 0" % minutes, u.telem.gaps == 0 and u.bad == 0 and
              u.telem.received > 60 * float(minutes) * 95, "alınan %d, boşluk %d, bozuk %d" % (u.telem.received, u.telem.gaps, u.bad))
        check("D6b TOPLAM en çok <= 2000 µs", u.whole.n > 0 and u.whole.max <= 2000, "en az, ort, en çok, jitter: %s" % (u.whole.cells()[:4],))
        check("D6c komut kaybı 0, satır kaybı 0", fails == 0 and source.lines.join.lost == lost0,
              "%d komut, cevapsız %d, kayıp datagram %d, host hata %s" % (
                  cmds, fails, source.lines.join.lost - lost0,
                  {k: v - err0.get(k, 0) for k, v in h.c.items() if ("bad" in k or "err" in k) and v != err0.get(k, 0)} or "yok"))
        print("    F746", u.f746.cells(), "M33", u.m33.cells())

        n = uart()
        ses.stop()
        t = wait(lambda: uart() > n, 5)
        check("D7a Durdur: satırlar UART'a döner (<= 2 s, 'st' saniyede bir)", t is not None and t <= 2.0, text(t))
        ses.start()
        wait(lambda: source.active() == src.MP257 and quiet(1.5) == 0, 12)
        ses.wanted = False
        n = uart()
        kill9()
        t = wait(lambda: uart() > n, 8)
        check("D7b kill -9: satırlar UART'a 4 s içinde döner", t is not None and t <= 4.0, text(t))
    finally:
        ses.stop()
        source.close()
        lk.close()
    print("SONUC:", "GECTI" if results and all(results) else "KALDI", "%d/%d" % (sum(results), len(results)))
    return 0 if results and all(results) else 1


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:3]))
