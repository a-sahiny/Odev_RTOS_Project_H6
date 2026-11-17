"""T07 Uzun çalışma (tasarım 11, 18.3, 20.2).

En az 30 dk seed'li karışık trafik. Her dilimin sonunda STOP -> boşalt -> görüntü -> START.
Bir dilimde host ARP isteklerine cevap vermez (arp_mute).
Beklenen: ısınma sonrası boştaki kaynak seviyeleri sabit (nb_free, pool'lar, heap düz),
kalıcı descriptor kaybı yok, EPOCH değişmiyor, her görüntüde muhasebe eşitlikleri tutuyor.
Kartta test edilen: sızıntı (heap, network buffer, desc / rapor pool'ları), ARP yaşlanması
ve yenilemesi, uzun süreli sayaç tutarlılığı, kendiliğinden reset olmaması.
Not: standart derlemede ARP kaydı ~25 dk yaşar; ARP yaşlanmasını kısa sürede görmek için
tasarım 11'deki test derlemesi (ipconfigMAX_ARP_AGE 12) gerekir.
"""

import random
import time

from vnet_host import accounting as acc, proto
from tests.common import TestResult, prepare, counters, check_equalities, stats_text


def run(h, minutes=30.0, slice_s=300.0, seed=707, rate_hz=20, arp_mute_slice=2, **_):
    res = TestResult("T07", "Uzun çalışma")
    res.meta.update(minutes=minutes, slice_s=slice_s, seed=seed, telem_hz=rate_hz, arp_mute_slice=arp_mute_slice)
    hello = prepare(h, res, rate=rate_hz)
    rng = random.Random(seed)
    slices = max(1, int(round(minutes * 60.0 / slice_s)))
    snaps = []
    c_prev = counters(h)
    rtts = []
    for s in range(slices):
        mute = (s == arp_mute_slice % slices) and slices > 1
        h.arp_mute = mute
        h.start()
        t_end = time.perf_counter() + slice_s
        nxt_cmd = time.perf_counter()
        while time.perf_counter() < t_end:
            k = rng.random()
            if k < 0.45:
                h.send_env([rng.randint(-2000, 2000) for _ in range(rng.randint(1, 8))])
            elif k < 0.9:
                h.send_bulk(rng.getrandbits(32), rng.randint(8, 120))
            else:
                h.send_cal([rng.randint(-100, 100) for _ in range(6)])
            if time.perf_counter() >= nxt_cmd:
                nxt_cmd += 1.0
                try:
                    rtts.append(h.command(proto.CMD_GET_STATS, proto.payload_get_stats(3), timeout=0.5).rtt_ms)
                except Exception:
                    res.info("dilim %d komut" % s, "cevapsız (ARP kesik: %s)" % mute)
            time.sleep(rng.uniform(0.05, 0.25))
        h.arp_mute = False
        h.stop()
        time.sleep(0.5)
        c = counters(h)
        st = h.state()
        d = acc.delta(c_prev, c)
        c_prev = c
        snaps.append({"slice": s, "arp_mute": mute, "st": st, "d": d})
        lbl = "dilim %d/%d%s" % (s + 1, slices, " (ARP cevapsız)" if mute else "")
        res.info(lbl + " kaynaklar", "nb=%s heap=%s cp=%s lp=%s rp=%s ep=%s" % (
            st.get("nb"), st.get("heap"), st.get("cp"), st.get("lp"), st.get("rp"), st.get("ep")) if st else "STATE yok")
        res.info(lbl + " ARP sayaçları", acc.nonzero(d, ("arp_delayed", "arp_delayed_full", "arp_expired",
                                                          "arp_tx_dropped")) or "yok")
        check_equalities(res, d, lbl)

    good = [x["st"] for x in snaps if x["st"]]
    if len(good) >= 2:
        warm = good[1:] if len(good) > 2 else good
        heap_vals = [g["heap"][0] for g in warm]
        nb_vals = [g["nb"][0] for g in warm]
        pools = [(g["cp"][0], g["lp"][0], g["rp"][0]) for g in warm]
        res.check("heap ısınma sonrası düz", "heap_free sabit", heap_vals, len(set(heap_vals)) == 1)
        res.check("network buffer kalıcı kaybı yok", "boşta nb_free sabit", nb_vals, len(set(nb_vals)) == 1)
        res.check("desc / rapor pool'ları boşta dolu", "cp, lp, rp sabit", pools, len(set(pools)) == 1)
    eps = {g.get("ep") for g in good} | {hello.get("ep")}
    res.check("kendiliğinden reset yok", "EPOCH tek değer", eps, len(eps) == 1)
    res.info("komut RTT (bütün çalışma)", stats_text(rtts))
    h.set_rate(proto.RATE_DEFAULT_HZ)
    return res
