"""T01 Parçalı giriş (tasarım 14, 20.2).

Aynı SET_RATE satırı tek yazımda, bayt bayt ve seed'li 1–17 baytlık parçalarla gönderilir;
komutun yalnız '\n'den sonra işlendiği, sonucun parçalamadan bağımsız olduğu kontrol edilir.
5 komut tek yazımda: sıra korunur. FRAME / FRAME 0 / FRAME 1 biçimlerinin üçü aynı sonucu verir.
Kartta test edilen: line_rx satır birleştirici (bayt bayt durum makinesi), vnetRx FCS bayrağı,
satır sayaçları (RX_FCS_NONE / RX_FCS_0 / RX_FCS_1).
"""

import time

from vnet_host import lines, proto
from tests.common import TestResult, prepare, counters


def run(h, seed=1234, **_):
    res = TestResult("T01", "Parçalı giriş")
    res.meta.update(seed=seed)
    prepare(h, res)

    # 1) Üç parçalama biçimi, her biri farklı hız değeri ve yeni REQ_ID ile
    for mode, hz in (("whole", 10), ("bytes", 20), ("random", 50)):
        req_id, pkt = h.command_packet(proto.CMD_SET_RATE, proto.payload_set_rate(hz))
        line = lines.frame_line(h.udp_frame(proto.PORT_CTRL, pkt), 1)
        with h._cv:
            h._pending[req_id] = [None]
        # Satır '\n'e kadar yarım bekliyor: bu sürede hattı ayır (rapor 092). Yoksa arka plandaki
        # başka bir satır (ör. ARP cevabı) araya girer ve kart ikisini tek, bozuk bir satır görür.
        with h.link.hold():
            h.link.write_fragmented(line[:-1], mode=mode, seed=seed, gap_s=0.003)
            t_before_nl = time.perf_counter()
            time.sleep(0.12)        # '\n' gelmeden cevap olmamalı (< 200 ms satır timeout'u)
            early = h._pending[req_id][0]
            h.link.write(b"\n")
        deadline = time.perf_counter() + 1.0
        while h._pending[req_id][0] is None and time.perf_counter() < deadline:
            time.sleep(0.01)
        resp = h._pending.pop(req_id)[0]
        res.check("SET_RATE %d Hz, yazım: %s (%d bayt)" % (hz, mode, len(line)),
                  "'\\n'den önce cevap yok; sonra OK, hız %d" % hz,
                  "erken cevap: %s; cevap: %s" % ("VAR" if early else "yok",
                                                  "%s hız %s" % (resp.status_name, resp.applied_rate) if resp else "YOK"),
                  early is None and resp is not None and resp.ok and resp.applied_rate == hz
                  and resp.rx_time > t_before_nl)

    # 2) 5 komut tek yazımda: sıra korunur
    reqs, blob = [], b""
    for hz in (10, 20, 50, 100, 200):
        req_id, pkt = h.command_packet(proto.CMD_SET_RATE, proto.payload_set_rate(hz))
        reqs.append((req_id, hz))
        with h._cv:
            h._pending[req_id] = [None]
        blob += lines.frame_line(h.udp_frame(proto.PORT_CTRL, pkt), 1)
    h.link.write(blob)
    deadline = time.perf_counter() + 2.0
    while any(h._pending[r][0] is None for r, _ in reqs) and time.perf_counter() < deadline:
        time.sleep(0.01)
    got = [(r, hz, h._pending.pop(r)[0]) for r, hz in reqs]
    ok_all = all(x[2] is not None and x[2].ok and x[2].applied_rate == x[1] for x in got)
    order = [x[2].seq for x in got if x[2] is not None]
    in_order = order == sorted(order) and len(order) == 5
    res.check("5 SET_RATE satırı tek yazımda (%d bayt)" % len(blob), "5 cevap OK, cevap SEQ'leri istek sırasında",
              "cevap %d/5, SEQ %s" % (len(order), order), ok_all and in_order)

    # 3) FRAME / FRAME 0 / FRAME 1: aynı sonuç, ayrı sayaçlar
    c0 = counters(h)
    results = []
    for flag, hz in ((None, 20), (0, 50), (1, 10)):
        # Biçim yalnız bu satıra (rapor 092): ortak h.fcs_flag değişseydi arka plandaki satırlar
        # (ARP cevabı) da o biçimle gider, Δrx_fcs_none / Δrx_fcs0 bozulurdu.
        results.append((flag, h.set_rate(hz, fcs_flag=flag)))
    c1 = counters(h)
    d = {k: (c1[k] - c0[k]) & 0xFFFFFFFF for k in ("rx_fcs_none", "rx_fcs0", "rx_fcs1", "cmd_new")}
    res.check("SET_RATE: 'FRAME <hex>', 'FRAME 0 <hex>', 'FRAME 1 <hex+FCS>'",
              "üçü de OK; Δrx_fcs_none=1, Δrx_fcs0=1, Δrx_fcs1≥1",
              "%s; Δ=%s" % (", ".join("%s:%s" % (f, r.status_name) for f, r in results), d),
              all(r.ok for _, r in results) and d["rx_fcs_none"] == 1 and d["rx_fcs0"] == 1 and d["rx_fcs1"] >= 1)
    h.set_rate(proto.RATE_DEFAULT_HZ)
    return res
