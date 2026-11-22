"""T04 Trafik patlaması (tasarım 8, 17, 18, 20.2).

Normal seviye: komut 5/s + ENV 5/s + BULK 5/s + CAL 1/s (≈16 frame/s) ve telemetri 20 Hz.
Seviyeler 1×, 2×, 4× (ve istenirse 8× = hat sınırının üstü). Her seviyenin sonunda STOP ->
boşalt -> GET_STATS: tasarım 18.2 eşitlikleri, komut RTT ve LAT_US dağılımı, telemetri
kaybı, EPOCH (reset yok).
Kartta test edilen: bütün RX yolu yük altında (vnetRx, IP task, iki lane), TX yolu (uart_tx
queue'su, kontrol rezervi), muhasebe; önemli lane gecikmesinin yükten bağımsızlığı.
"""

import random
import threading
import time

from vnet_host import accounting as acc, proto
from tests.common import TestResult, prepare, counters, check_equalities, stats_text


def _level(h, res, mult, seconds, rng, rate_hz):
    rates = {"env": 5 * mult, "bulk": 5 * mult, "cal": 1 * mult}
    cmd_rate = 5 * mult
    c0 = counters(h)
    tx0, telem0, gaps0 = c0["_host_tx"], h.telem.received, h.telem.gaps
    h.set_rate(rate_hz)
    h.start()
    rtts, lats, fails = [], [], [0]
    stop = threading.Event()

    def commands():
        period = 1.0 / cmd_rate
        nxt = time.perf_counter()
        while not stop.is_set():
            try:
                r = h.command(proto.CMD_GET_STATS, proto.payload_get_stats(3), timeout=0.4, retries=1)
                rtts.append(r.rtt_ms)
                lats.append(r.lat_us / 1000.0)
            except Exception:
                fails[0] += 1
            nxt += period
            time.sleep(max(0.0, nxt - time.perf_counter()))

    th = threading.Thread(target=commands, daemon=True)
    th.start()
    sent = {"env": 0, "bulk": 0, "cal": 0}
    nxt = {k: time.perf_counter() for k in rates}
    t_end = time.perf_counter() + seconds
    while time.perf_counter() < t_end:
        now = time.perf_counter()
        for k, r in rates.items():
            if now >= nxt[k]:
                nxt[k] += 1.0 / r
                if k == "env":
                    h.send_env([rng.randint(-3000, 3000) for _ in range(rng.randint(1, 8))])
                elif k == "bulk":
                    h.send_bulk(rng.getrandbits(32), rng.randint(16, 196))
                else:
                    h.send_cal([rng.randint(-200, 200) for _ in range(6)])
                sent[k] += 1
        time.sleep(0.002)
    stop.set()
    th.join(2.0)
    h.stop()
    time.sleep(0.3)
    c1 = counters(h)
    d = acc.delta(c0, c1)
    lbl = "%d× (%ds)" % (mult, seconds)
    offered = sum(sent.values()) / seconds + cmd_rate
    res.info("%s gönderilen" % lbl, "%s, komut %d (%.1f frame/s sunulan)" % (sent, len(rtts) + fails[0], offered))
    res.info("%s hat: host→kart" % lbl, "%.0f bayt/s (%d baud ≈ %d bayt/s)" %
             ((h.link.tx_bytes - tx0) / seconds, getattr(h.link, "baud", 1000000), getattr(h.link, "baud", 1000000) // 10))
    # RTT seri hattı da içerir (hat dolunca komut satırı veri satırlarının arkasında bekler);
    # önemli lane'in kart içi gecikmesi LAT_US'tur ve seviyeden bağımsız kalmalıdır.
    if mult <= 2:
        res.check("%s komutlar" % lbl, "hepsi cevaplanır", "%s, cevapsız %d" % (stats_text(rtts), fails[0]),
                  fails[0] == 0 and bool(rtts))
    else:
        res.info("%s komut RTT (hat dolu olabilir)" % lbl, "%s, cevapsız %d" % (stats_text(rtts), fails[0]))
    res.check("%s LAT_US (kart içi M1→M3)" % lbl, "en kötü < 10 ms", stats_text(lats),
              bool(lats) and max(lats) < 10.0)
    res.info("%s telemetri" % lbl, "alınan %d, SEQ boşluğu %d, üretilen %d (Δtelem_gen)" % (
        h.telem.received - telem0, h.telem.gaps - gaps0, d["telem_gen"]))
    res.info("%s ret / düşme sayaçları" % lbl, acc.nonzero(d, (
        "uart_rx_sb_ovf", "uart_rx_dma_lag", "uart_ore", "rx_no_netbuf", "rx_ipq_full", "cb_crit_sb_full",
        "cb_low_sb_full", "pool_crit_empty", "pool_low_empty", "ctrlq_full", "workq_full", "txq_full",
        "txq_reserve", "arp_delayed_full", "seq_gap_low")) or "yok")
    check_equalities(res, d, lbl, host_tx_bytes=c1["_host_tx"] - tx0)
    return rtts


def run(h, levels=(1, 2, 4), seconds=10, seed=404, rate_hz=20, **_):
    res = TestResult("T04", "Trafik patlaması")
    res.meta.update(levels=list(levels), seconds=seconds, seed=seed, telem_hz=rate_hz)
    hello = prepare(h, res)
    rng = random.Random(seed)
    all_rtts = {}
    for m in levels:
        all_rtts[m] = _level(h, res, m, seconds, rng, rate_hz)
    st = h.state()
    res.check("reset / kilitlenme yok", "EPOCH aynı (%s), kart cevap veriyor" % hello.get("ep"),
              "EPOCH %s" % (st.get("ep") if st else "cevap yok"), bool(st) and st.get("ep") == hello.get("ep"))
    if len(all_rtts) > 1 and all(all_rtts.values()):
        means = {m: sum(v) / len(v) for m, v in all_rtts.items()}
        res.info("komut RTT ortalaması seviyeye göre", ", ".join("%d×: %.1f ms" % kv for kv in means.items()))
    h.set_rate(proto.RATE_DEFAULT_HZ)
    return res
