"""T08 IMU hızları, açılar ve TELEM gecikmesi.

Kart her IMU örneğinde (DRDY kesmesi) bir TELEM yollar; host hiçbir şey sormaz. SET_RATE IMU'nun
örnekleme hızını değiştirir: 10 / 20 / 50 / 100 / 200 Hz (başka değer E_RANGE). Her TELEM ham ivme /
jiroskop, roll / pitch / yaw ve bir önceki TELEM'in kartta ölçülen gecikmesini taşır:
DRDY (IMU verisi hazır) -> o TELEM satırının son biti UART'tan çıktı (TX bitti kesmesi).
Hedef 10 ms, deadline 12 ms (kart sayaçları telem_over10 / telem_miss). PC / USB gecikmesi
hiçbir ölçüme girmez. TELEM satırı 1 Mbaud'da 167 karakter = 1,67 ms (≤ 2 ms şartı).
Kartta test edilen: imu_spi.c Imu_SetRate (SMPLRT_DIV + DLPF), meas_task.c hıza bağlı dt / filtre,
uart_tx.c TELEM'i queue'nun önüne alma ve DRDY -> M6 ölçümü.
"""

import time

from vnet_host import proto
from vnet_host import accounting as acc
from vnet_host.hoststack import TelemLatencyStats
from tests.common import TestResult, prepare, counters

LINE_CHARS = 8 + 2 * (14 + 20 + 8 + proto.OVERHEAD_BYTES + proto.TELEM_LEN + 4) + 1


def run(h, seconds=10, **_):
    seconds = float(seconds)
    per_rate = max(2.0, seconds / len(proto.RATES_HZ))
    res = TestResult("T08", "IMU hızları, açılar ve TELEM gecikmesi")
    res.meta["seconds_per_rate"] = per_rate
    prepare(h, res)
    c0 = counters(h)

    # 1) STOP durumunda TELEM gelmez
    rx0 = h.telem.received
    time.sleep(0.3)
    res.check("STOP durumunda 0,3 s", "TELEM gelmez", "%d TELEM" % (h.telem.received - rx0), h.telem.received == rx0)

    # 2) eski GET_IMU (0x05) artık komut değil; listede olmayan hız reddedilir
    r = h.command(0x05, b"", timeout=0.5)
    res.check("eski GET_IMU (TYPE 0x05)", "E_TYPE", r.status_name, r.status == proto.ST_E_TYPE)
    r = h.set_rate(30)
    res.check("SET_RATE 30 (listede yok)", "E_RANGE, hız değişmez", "%s, hız %s" % (r.status_name, r.applied_rate),
              r.status == proto.ST_E_RANGE)

    # 3) TELEM satır süresi (hesap): 'FRAME 1 ' + hex(Ethernet + IPv4 + UDP + paket + FCS) + '\n'
    baud = getattr(h.link, "baud", 1000000)
    line_ms = LINE_CHARS * 10 * 1000.0 / baud
    res.check("TELEM satırı (%d karakter, %d baud)" % (LINE_CHARS, baud), "≤ 2 ms", "%.2f ms" % line_ms, line_ms <= 2.0)

    # 4) her hız: hız, kayıp, açılar, kart içi gecikme
    for hz in proto.RATES_HZ:
        r = h.set_rate(hz)
        lat = TelemLatencyStats()
        got = []

        def on(kind, t):
            if kind == "telem":
                lat.add(t)
                got.append(t)
        h.listeners.append(on)
        rs = h.start()
        time.sleep(per_rate)
        h.stop()
        time.sleep(0.2)
        h.listeners.remove(on)
        span = (got[-1].rx_time - got[0].rx_time) if len(got) > 1 else 0.0
        meas_hz = (len(got) - 1) / span if span > 0 else 0.0
        gaps = sum((((b.seq - a.seq) & 0xFFFF) - 1) for a, b in zip(got, got[1:]) if ((b.seq - a.seq) & 0xFFFF) < 0x8000)
        res.check("%d Hz: SET_RATE + START, %.0f s" % (hz, per_rate),
                  "OK; TELEM hızı %d Hz ±10%%, SEQ boşluğu yok" % hz,
                  "%s / %s; %d TELEM, %.1f Hz, boşluk %d" % (r.status_name, rs.status_name, len(got), meas_hz, gaps),
                  r.ok and rs.ok and abs(meas_hz - hz) <= hz * 0.1 and gaps == 0)
        in_range = all(-180.0 <= t.roll <= 180.0 and -90.0 <= t.pitch <= 90.0 and -180.0 <= t.yaw <= 180.0 for t in got)
        first_ready = next((i for i, t in enumerate(got) if t.bias_ready), None)
        res.check("%d Hz: açılar" % hz, "aralıkta; BIAS_READY ~1 s sonra (≈ %d örnek)" % hz,
                  "BIAS_READY ilk %s. örnekte; son: roll %.2f pitch %.2f yaw %.2f" % (
                      first_ready, got[-1].roll if got else 0, got[-1].pitch if got else 0, got[-1].yaw if got else 0),
                  in_range and first_ready is not None and abs(first_ready - hz) <= max(2, hz // 10))
        s = lat.summary()
        res.check("%d Hz: kart DRDY → TX bitti (her TELEM bir öncekininkini taşır)" % hz, "ölçüm var, hepsi ≤ 12 ms",
                  "%d ölçüm; ort %.2f, p99 %.2f, en çok %.2f ms; ≤10 ms %d, >12 ms %d, bir öncekinden eski %d" % (
                      s["n"], s["mean_ms"], s["p99_ms"], s["max_ms"], s["le_10ms"], s["over_12ms"], s["behind"]),
                  s["n"] > 0 and s["over_12ms"] == 0)

    # 5) kartın kendi sayaçları
    c1 = counters(h)
    d = acc.delta(c0, c1)
    res.check("kart içi deadline (DRDY → TX bitti ≤ 12 ms)", "telem_miss artmaz",
              "Δtelem_miss=%d, Δtelem_over10=%d (hedef 10 ms aşımı, bilgi), Δtelem_gen=%d, Δimu_fail_run=%d" % (
                  d["telem_miss"], d["telem_over10"], d["telem_gen"], d["imu_fail_run"]),
              d["telem_miss"] == 0)
    res.check("L6: telem_gen = telem_sendto_ok + telem_sendto_fail", "eşit, sendto_fail 0",
              "%d = %d + %d" % (d["telem_gen"], d["telem_sendto_ok"], d["telem_sendto_fail"]),
              d["telem_gen"] == d["telem_sendto_ok"] + d["telem_sendto_fail"] and d["telem_sendto_fail"] == 0)
    h.set_rate(proto.RATE_DEFAULT_HZ)
    res.note("Gecikme yalnız kart zamanıyla: DRDY kesmesi (IMU verisi hazır) -> TELEM satırının son biti UART'tan "
             "çıktı (TX bitti kesmesi). Bir TELEM'in süresi o gittikten sonra bilinir, bir sonraki TELEM'de gelir.")
    return res
