"""T03 Yavaş tüketici (tasarım 9, 17, 20.2).

HOLD 5000 ile lowParser 5 s bekletilir (yapay enjeksiyon), bu sırada 5001'e 50 geçerli
BULK paketi gönderilir (network buffer pool'unun 3 katı). Aynı anda önemli lane'e komutlar.
Beklenen: sbLow'a sığmayanlar CB_LOW_SB_FULL ile sayılıp düşer, network buffer'lar hemen
pool'a döner (nb_free boştaki değerinde), HOLD bitince sbLow'daki record'lar işlenir,
önemli lane bu sırada normal hızda cevap verir.
Kartta test edilen: udp_in.c callback'i (yer kontrolü, tek seferde yazma), sbLow MessageBuffer,
lowParser HOLD, network buffer pool'u, önemli / önemsiz lane ayrımı.
"""

import time

from vnet_host import proto
from tests.common import TestResult, prepare, counters, check_equalities, stats_text
from vnet_host import accounting as acc


def run(h, hold_ms=5000, packets=50, bulk_bytes=150, seed=303, **_):
    res = TestResult("T03", "Yavaş tüketici")
    res.meta.update(hold_ms=hold_ms, packets=packets, bulk_bytes=bulk_bytes, seed=seed)
    prepare(h, res)
    st0 = h.state()
    nb_idle = st0["nb"][0] if st0 else None
    # SEQ eşitleme (rapor 092): kart BULK kanalının son SEQ'ini önceki host sürecinden hatırlar; bu
    # süreç SEQ'i baştan başlatınca ilk paketler "eski" sayılıp tasarım gereği atılır (P2: 3. eski
    # paket RESYNC + kabul). Ölçümden önce 3 paket: kanalın tabanı bu sürecin SEQ'ine gelir.
    for i in range(3):
        h.send_bulk(seed - 3 + i, bulk_bytes)
    time.sleep(0.2)
    c0 = counters(h)
    tx0 = c0["_host_tx"]

    ack = h.hold(hold_ms)
    t_hold = time.perf_counter()
    res.check("HOLD %d" % hold_ms, "ack hold_requested", ack and ack.get("result"),
              bool(ack) and ack.get("result") == "hold_requested", inj="HOLD %d" % hold_ms)

    # HOLD lowParser'ın bir sonraki record'unda başlar: tetiklemek için tek paket
    h.send_bulk(seed, bulk_bytes)
    rtts, lats = [], []
    for i in range(packets - 1):
        h.send_bulk(seed + 1 + i, bulk_bytes)
        if i % 10 == 9:
            r = h.command(proto.CMD_GET_STATS, proto.payload_get_stats(3))
            rtts.append(r.rtt_ms)
            lats.append(r.lat_us / 1000.0)
    send_s = time.perf_counter() - t_hold
    st_hold = h.state()
    res.info("50 paket gönderim süresi", "%.2f s (HOLD %d ms içinde: %s)" % (send_s, hold_ms,
                                                                          "evet" if send_s * 1000 < hold_ms else "HAYIR"))
    if st_hold:
        res.check("HOLD sırasında STATE 'inj'", "bit0 (HOLD) set", st_hold.get("inj"),
                  bool(st_hold.get("inj", [0])[0] & 1), inj="HOLD")
        res.info("HOLD sırasında nb (boş / en az)", "%s, boşta %s" % (st_hold.get("nb"), nb_idle), inj="HOLD")
        res.info("HOLD sırasında sbLow en çok doluluk (bayt)", st_hold.get("sb", [None] * 3)[2], inj="HOLD")
    res.check("önemli lane HOLD sırasında (GET_STATS RTT)", "cevaplar gelir, RTT < 160 ms", stats_text(rtts),
              bool(rtts) and max(rtts) < 160, inj="HOLD")
    res.info("önemli lane LAT_US (kart içi M1→M3)", stats_text(lats), inj="HOLD")

    left = hold_ms / 1000.0 - (time.perf_counter() - t_hold)
    if left > 0:
        time.sleep(left)
    time.sleep(1.0)
    c1 = counters(h)
    st1 = h.state()
    d = acc.delta(c0, c1)
    res.info("önemsiz lane yolu (record → paket → worker)", ", ".join(
        "%s=%d" % (k, d[k]) for k in ("rec_read_low", "pkt_ok_low", "seq_dup_low", "seq_gap_low", "seq_old_low",
                                       "seq_resync_low", "evt_posted_low", "pool_low_empty", "workq_full",
                                       "bulk_pkts")))
    res.check("callback: 5001'e gelen datagram", "Δcb_low = %d" % packets, "Δcb_low = %d" % d["cb_low"],
              d["cb_low"] == packets)
    res.check("sbLow'a sığmayanlar sayılır", "Δsb_written_low + Δcb_low_sb_full = %d, Δcb_low_sb_full > 0" % packets,
              "%d + %d" % (d["sb_written_low"], d["cb_low_sb_full"]),
              d["sb_written_low"] + d["cb_low_sb_full"] == packets and d["cb_low_sb_full"] > 0, inj="HOLD")
    res.check("HOLD bitince sbLow'daki record'lar işlenir", "Δbulk_pkts = Δsb_written_low, Δbulk_bad = 0",
              "Δbulk_pkts=%d, Δsb_written_low=%d, Δbulk_bad=%d" % (d["bulk_pkts"], d["sb_written_low"], d["bulk_bad"]),
              d["bulk_pkts"] == d["sb_written_low"] and d["bulk_bad"] == 0)
    if st1 and nb_idle is not None:
        res.check("network buffer pool'u", "nb_free boştaki değerine döner (%d)" % nb_idle,
                  "nb=%s" % st1.get("nb"), st1["nb"][0] == nb_idle)
        res.info("network buffer en az boş (açılıştan beri)", st1["nb"][1])
    check_equalities(res, d, "HOLD sonrası, sakin", host_tx_bytes=c1["_host_tx"] - tx0)
    return res
