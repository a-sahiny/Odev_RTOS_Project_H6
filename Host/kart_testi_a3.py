r"""6. madde, 3. aşama kart testi: Linux okurken PC'nin komutları da Linux üzerinden F746'ya gider.

Çalıştırma: python Host\kart_testi_a3.py COM6 [dakika]   (TelemetriHost kapalı: UDP 5002, 5003 ve seri port boş)
E6 [dakika] sürer (varsayılan 5). Kalıcı değişiklik yapmaz; sonunda CAL yollamaz, hız 100 Hz, STOP. E7 telem_udp'yi öldürür.
"""
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tests.common import counters  # noqa: E402
from vnet_host import accounting as acc, link, proto, source as src  # noqa: E402
from vnet_host.hoststack import Host  # noqa: E402
from kart_testi_6 import check, kill9, results, text, wait  # noqa: E402


def main(port="COM6", minutes="5"):
    h = Host(link.SerialLink(port, 1000000))
    lk = h.link
    source = src.TelemSource()
    source.on_line = lk.feed_line
    lk.route = source.send_command
    ses = src.BoardSession(source)
    serial_tx = lambda: lk.tx_bytes - lk.linux_tx                    # noqa: E731
    serial_rx = lambda: lk.rx_lines - lk.linux_lines                 # noqa: E731
    stats = lambda: h.command(proto.CMD_GET_STATS, proto.payload_get_stats(3), timeout=0.5, retries=0)  # noqa: E731

    try:
        h.connect()
        check("E5a Linux yokken komutlar UART'tan", h.set_rate(100).ok and h.stop().ok and lk.linux_tx == 0,
              "seri %d bayt, Linux %d" % (serial_tx(), lk.linux_tx))
        error = ses.start()
        wait(lambda: source.active() == src.MP257 and lk.linux_lines > 0, 12)
        time.sleep(3)                                                # F746 "Linux var"ı alsın, UART'taki son satır bitsin
        tx0, rx0 = serial_tx(), serial_rx()
        seen = []
        for name, fn in (("HELLO", h.hello), ("STATE", h.state), ("ping", h.ping),
                         ("SET_RATE 20", lambda: h.set_rate(20).ok), ("SET_RATE 100", lambda: h.set_rate(100).ok),
                         ("START", lambda: h.start().ok), ("STOP", lambda: h.stop().ok),
                         ("GET_STATS", lambda: h.get_counters()), ("SET_RATE 77 ret", lambda: not h.set_rate(77).ok)):
            try:
                seen.append((name, bool(fn())))
            except Exception as e:
                seen.append((name, str(e)))
        check("E5b bütün komutlar Linux'tan, seri porta 0 bayt", not error and all(v is True for _, v in seen)
              and serial_tx() == tx0 and serial_rx() == rx0,
              "seri tx %d rx %d, Linux tx %d %s" % (serial_tx() - tx0, serial_rx() - rx0, lk.linux_tx,
                                                    [x for x in seen if x[1] is not True] or ""))
        rtts, fails = [], 0
        for _ in range(100):
            try:
                rtts.append(stats().rtt_ms)
            except Exception:
                fails += 1
        check("E5c 100 GET_STATS, tekrar denemesiz", fails == 0, "cevapsız %d, RTT ort %.1f en çok %.1f ms" % (
            fails, sum(rtts) / max(1, len(rtts)), max(rtts or [0])))

        rng = random.Random(3)
        c0 = counters(h)
        source.udp.reset()
        lost0, err0 = source.lines.join.lost, dict(h.c)
        end = time.perf_counter() + 60 * float(minutes)
        cmds = fails = env = bulk = 0
        while time.perf_counter() < end:                             # saniyede 10 komut, 5 ENV, 5 BULK
            try:
                stats()
            except Exception:
                fails += 1
            cmds += 1
            if cmds % 2:
                h.send_env([rng.randint(-3000, 3000) for _ in range(rng.randint(1, 8))])
                env += 1
            else:
                h.send_bulk(rng.getrandbits(32), rng.randint(16, 196))
                bulk += 1
            time.sleep(0.09)
        u = source.udp
        check("E6a %s dk: TELEM kaybı 0" % minutes, u.telem.gaps == 0 and u.bad == 0 and u.telem.received > 60 * float(minutes) * 95,
              "alınan %d, boşluk %d, bozuk %d" % (u.telem.received, u.telem.gaps, u.bad))
        check("E6b TOPLAM en çok <= 2000 µs", u.whole.n > 0 and u.whole.max <= 2000, "en az, ort, en çok, jitter: %s" % (u.whole.cells()[:4],))
        check("E6c komut ve satır kaybı 0, seri porta 0 bayt", fails == 0 and source.lines.join.lost == lost0 and serial_tx() == tx0,
              "%d komut, cevapsız %d, kayıp datagram %d, seri tx %d, host hata %s" % (
                  cmds, fails, source.lines.join.lost - lost0, serial_tx() - tx0,
                  {k: v - err0.get(k, 0) for k, v in h.c.items() if ("bad" in k or "err" in k) and v != err0.get(k, 0)} or "yok"))
        c1 = counters(h)
        d = acc.delta(c0, c1)
        bad = [c for c in acc.equalities(d, host_tx_bytes=c1["_host_tx"] - c0["_host_tx"]) if not c.ok]
        # Yeni Host SEQ'i baştan sayar: kart önceki koşudan kalan SEQ yüzünden ilk 2 + 2 paketi reddedip eşitlenir (UART'ta da)
        done = d["env_pkts"] + d["bulk_pkts"] + d["seq_old_low"]
        check("E6d tasarım 18.2 eşitlikleri, ENV + BULK kaybı 0", not bad and done == env + bulk and d["seq_old_low"] <= 4,
              "%s; işlenen ENV %d BULK %d + eski SEQ %d, gönderilen %d" % ("hepsi tutar" if not bad else "; ".join("%s %s (%d≠%d)" % (c.layer, c.text, c.lhs, c.rhs) for c in bad),
                                            d["env_pkts"], d["bulk_pkts"], d["seq_old_low"], env + bulk))

        ses.stop()
        t = wait(lambda: h.hello(timeout=0.5) is not None and serial_rx() > rx0, 5)
        check("E7a Durdur: komut UART'tan cevaplanır", t is not None and t <= 4.0 and serial_tx() > tx0, text(t))
        ses.start()
        wait(lambda: source.active() == src.MP257 and h.hello(timeout=0.5) is not None, 12)
        time.sleep(3)
        ses.wanted = False
        rx0 = serial_rx()
        t0 = time.perf_counter()
        kill9()
        t = wait(lambda: h.hello(timeout=0.5) is not None and serial_rx() > rx0, 8)
        check("E7b kill -9: 4 s içinde UART'tan cevap (ssh dahil)", t is not None and time.perf_counter() - t0 <= 4.0 + 1.0, text(t))
        check("E7c sonunda STOP, 100 Hz", h.stop().ok and h.set_rate(100).ok, "OK")
    finally:
        ses.stop()
        source.close()
        lk.close()
    print("SONUC:", "GECTI" if results and all(results) else "KALDI", "%d/%d" % (sum(results), len(results)))
    return 0 if results and all(results) else 1


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:3]))
