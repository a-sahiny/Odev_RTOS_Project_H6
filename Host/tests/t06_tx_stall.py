"""T06 TX tıkanması (tasarım 10, 13.3, 20.2).

A) Host okumayı keser (akış kontrolü yok): kart bunu göremez, birikme kartta GÖZLENMEZ;
   kayıp host tarafında telemetri SEQ boşluğu olarak görünür (USB / VCP katmanı).
B) TXDELAY 50 (yapay enjeksiyon): uartTx her öğeden sonra 50 ms bekler, TX queue'su dolar.
   Beklenen: TXQ_FULL / TXQ_RESERVE artar, IP task bloklanmaz (RX ilerler: to_stack),
   komut cevapları kontrol rezervi sayesinde yine gelir; enjeksiyon kalkınca nb_free döner.
Kartta test edilen: uart_tx.c prvAdmit (son 2 yer kontrol cevaplarına), prvVNetOutput'un
0 beklemesi, uartTx TXDELAY, network buffer'ların ret yolunda bırakılması.
"""

import time

from vnet_host import accounting as acc, proto
from tests.common import TestResult, prepare, counters, check_equalities, stats_text


def run(h, pause_s=3.0, txdelay_ms=50, load_s=4.0, rate_hz=50, seed=606, **_):
    res = TestResult("T06", "TX tıkanması")
    res.meta.update(pause_s=pause_s, txdelay_ms=txdelay_ms, load_s=load_s, telem_hz=rate_hz, seed=seed)
    prepare(h, res, rate=rate_hz)
    st0 = h.state()
    nb_idle = st0["nb"][0] if st0 else None

    # A) host okumayı keser
    c0 = counters(h)
    h.start()
    time.sleep(1.0)
    gaps0, rx0 = h.telem.gaps, h.telem.received
    h.link.pause_reading(True)
    time.sleep(pause_s)
    h.link.pause_reading(False)
    time.sleep(1.0)
    h.stop()
    c1 = counters(h)
    d = acc.delta(c0, c1)
    res.check("A) host %.1f s okumadı" % pause_s, "kartta birikme yok: Δtxq_full = 0, Δtxq_reserve = 0",
              "Δtxq_full=%d, Δtxq_reserve=%d, Δtx_lines=%d" % (d["txq_full"], d["txq_reserve"], d["tx_lines"]),
              d["txq_full"] == 0 and d["txq_reserve"] == 0)
    res.info("A) host tarafı kayıp (USB / VCP)", "telemetri alınan %d, SEQ boşluğu %d" % (
        h.telem.received - rx0, h.telem.gaps - gaps0))

    # B) TXDELAY ile TX queue'su doldurulur
    c0 = counters(h)
    ack = h.txdelay(txdelay_ms)
    res.check("TXDELAY %d" % txdelay_ms, "ack txdelay_set", ack and ack.get("result"),
              bool(ack) and ack.get("result") == "txdelay_set", inj="TXDELAY %d" % txdelay_ms)
    h.start()
    rtts, lats, fails = [], [], 0
    t_end = time.perf_counter() + load_s
    i = 0
    while time.perf_counter() < t_end:
        h.send_env([i, -i])
        i += 1
        try:
            r = h.command(proto.CMD_SET_RATE, proto.payload_set_rate(rate_hz), timeout=1.0, retries=1)
            rtts.append(r.rtt_ms)
            lats.append(r.lat_us / 1000.0)
        except Exception:
            fails += 1
        time.sleep(0.1)
    st_mid = h.state(timeout=2.0)
    h.stop()
    h.txdelay(0)
    time.sleep(1.5)
    c1 = counters(h)
    st1 = h.state()
    d = acc.delta(c0, c1)
    inj = "TXDELAY %d" % txdelay_ms
    res.check("B) TX queue'su dolar", "Δtxq_full + Δtxq_reserve > 0",
              "Δtxq_full=%d, Δtxq_reserve=%d, Δif_out=%d" % (d["txq_full"], d["txq_reserve"], d["if_out"]),
              d["txq_full"] + d["txq_reserve"] > 0, inj=inj)
    res.check("B) IP task bloklanmaz: RX ilerler", "Δto_stack > 0, Δrx_ipq_full = 0",
              "Δto_stack=%d, Δrx_ipq_full=%d, Δrx_no_netbuf=%d" % (d["to_stack"], d["rx_ipq_full"], d["rx_no_netbuf"]),
              d["to_stack"] > 0 and d["rx_ipq_full"] == 0, inj=inj)
    res.check("B) komut cevapları (kontrol rezervi)", "cevaplar gelir", "%s, cevapsız %d" % (stats_text(rtts), fails),
              bool(rtts) and fails <= len(rtts) // 10, inj=inj)
    res.info("B) LAT_US", stats_text(lats), inj=inj)
    if st_mid:
        res.info("B) yük altında STATE", "inj=%s q(TX en az boş)=%s nb=%s" % (st_mid.get("inj"), st_mid.get("q"),
                                                                           st_mid.get("nb")), inj=inj)
    if st1 and nb_idle is not None:
        res.check("enjeksiyon kalkınca network buffer'lar döner", "nb_free = %d" % nb_idle, "nb=%s inj=%s" % (
            st1.get("nb"), st1.get("inj")), st1["nb"][0] == nb_idle and st1["inj"][0] == 0)
    res.info("sayaç görüntüsü yeniden alındı (bir sayfa cevabı tekrarlandı)", "önce %d, sonra %d; host rx_bad %s" % (
        c0.get("_retakes", 0), c1.get("_retakes", 0),
        {k: v for k, v in h.c.items() if k.startswith("rx_bad") and v} or "yok"))
    check_equalities(res, d, "TXDELAY sonrası, sakin")
    h.set_rate(proto.RATE_DEFAULT_HZ)
    return res
