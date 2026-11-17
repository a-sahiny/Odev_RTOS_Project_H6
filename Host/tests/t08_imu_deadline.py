"""T08 IMU açıları ve GET_IMU deadline'ı (rapor 086).

GET_IMU önemli lane'de (port 5000) START / STOP gibi bir komuttur: kart en son IMU örneğini
roll / pitch / yaw ile cevaplar. Hedef 10 ms, deadline 12 ms; kart bunu kendi içinde ölçer:
istek UDP callback'ine geldi (M1) -> cevap satırının son baytı UART'tan çıktı (M6).
Kartta test edilen: cmd.c GET_IMU (yalnız START sonrası, tekrar penceresine girmez),
meas_task.c açı hesabı, uart_tx.c kontrol cevabını queue'nun önüne alma ve M1 -> M6 ölçümü
(imu_reply_over10 / imu_reply_miss sayaçları).

Deadline yalnız KART ZAMANIYLA değerlendirilir (rapor 091): her GET_IMU cevabı bir öncekinin kartta
ölçülen M1 -> M6 süresini taşır; Windows / USB ve host'un gecikmesi hiçbir satıra girmez.
İstek periyodu 20 ms (rapor 091). İzleme sırasında GET_STATS gönderilmez (uzun satır, IMU'yu bekletir).
"""

import time

from vnet_host import proto
from vnet_host import accounting as acc
from vnet_host.hoststack import ImuPoller, StatusError
from tests.common import TestResult, prepare, counters


def run(h, seconds=10, **_):
    seconds = float(seconds)
    res = TestResult("T08", "IMU açıları ve GET_IMU deadline'ı")
    res.meta["seconds_polled"] = seconds
    prepare(h, res, rate=10)
    c0 = counters(h)

    # 1) START'tan önce: E_STATE
    try:
        h.get_imu(timeout=0.5)
        st = "OK"
    except StatusError as e:
        st = proto.STATUS_NAMES.get(e.status, str(e.status))
    res.check("GET_IMU STOP durumunda", "E_STATE (ölçüm yok)", st, st == "E_STATE")

    # 2) yanlış LEN: E_LEN
    r = h.command(proto.CMD_GET_IMU, payload=b"\x00", timeout=0.5)
    res.check("GET_IMU LEN=1", "E_LEN, yalnız ortak kısım", "%s, ek %d B" % (r.status_name, len(r.extra)),
              r.status == proto.ST_E_LEN and len(r.extra) == 0)

    # 3) START, ilk örnek
    rs = h.start()
    first = None
    t_start = time.perf_counter()
    while time.perf_counter() - t_start < 1.0 and first is None:
        try:
            first = h.get_imu(timeout=0.2)
        except StatusError:
            time.sleep(0.005)
    res.check("START sonrası ilk GET_IMU", "OK, örnek no küçük, bias henüz ölçülüyor",
              "%s; idx %s, flags %s" % (rs.status_name, first.index if first else "-",
                                        first.flags if first else "-"),
              rs.ok and first is not None and first.index < 20 and not first.bias_ready)

    # 4) 100 Hz istek
    poller = ImuPoller(h, period_ms=proto.IMU_PERIOD_MS, timeout_ms=50.0)
    samples = []
    poller.on_sample = samples.append
    # Başka bir GET_IMU istemcisi (GUI izlemesi) bu sürede bekler (rapor 092): deadline tek
    # istemci için tanımlı. İki istemci aynı anda isterse cevaplardan biri diğerinin satırını da
    # bekler; önde 321 karakterlik bir rapor varsa ≈ 5,1 + 3,25 + 3,25 + 1,5 ≈ 13 ms (sağlanamaz).
    # Önce diğeri bekletilir, sonra bu poller başlar: hiçbir an iki istemci birlikte sormaz.
    with h.quiesce(exempt=poller):
        poller.start()
        time.sleep(seconds)
        poller.stop()
    s = poller.stats.summary()
    res.info("%.0f s, %.0f ms periyotla GET_IMU" % (seconds, proto.IMU_PERIOD_MS),
             "istek %d, OK %d, zaman aşımı %d, hata %s" % (s["requests"], s["ok"], s["timeouts"], s["errors"]))
    mono = all(((b.index - a.index) & 0xFFFFFFFF) < 0x80000000 for a, b in zip(samples, samples[1:]))
    res.check("örnek numaraları", "geri gitmez", "aynı örnek %d kez, atlanan %d" % (s["same_index"], s["index_gaps"]),
              mono)
    in_range = all(-180.0 <= x.roll <= 180.0 and -90.0 <= x.pitch <= 90.0 and -180.0 <= x.yaw <= 180.0 for x in samples)
    biased = [x for x in samples if x.bias_ready]
    res.check("açılar", "roll / yaw ±180°, pitch ±90° içinde; 1 s sonra BIAS_READY",
              "%d örnek, BIAS_READY %d; son: roll %.2f pitch %.2f yaw %.2f" % (
                  len(samples), len(biased), samples[-1].roll if samples else 0, samples[-1].pitch if samples else 0,
                  samples[-1].yaw if samples else 0),
              in_range and bool(biased))
    res.check("kart zamanıyla M1 -> M6 (her cevap bir öncekinin süresini taşır)",
              "en az bir ölçüm, hepsi ≤ 12 ms",
              "%d ölçüm; ort %.2f, p99 %.2f, en çok %.2f ms; ≤10 ms %d, ≤12 ms %d, >12 ms %d" % (
                  s["board_n"], s["board_mean_ms"], s["board_p99_ms"], s["board_max_ms"], s["le_10ms"], s["le_12ms"],
                  s["over_12ms"]),
              s["board_n"] > 0 and s["over_12ms"] == 0)
    res.info("M1 -> M6'nın parçaları (kart)", "işleme M1 -> M3 en çok %.2f ms, gönderim M3 -> M6 en çok %.2f ms" % (
        s["proc_max_ms"], s["send_max_ms"]))

    # 5) STOP, sonra E_STATE
    h.stop()
    try:
        h.get_imu(timeout=0.5)
        st = "OK"
    except StatusError as e:
        st = proto.STATUS_NAMES.get(e.status, str(e.status))
    res.check("STOP sonrası GET_IMU", "E_STATE", st, st == "E_STATE")

    # 6) kartın kendi deadline ölçümü
    c1 = counters(h)
    d = acc.delta(c0, c1)
    res.check("kart içi deadline (M1 -> M6 ≤ 12 ms)", "imu_reply_miss artmaz",
              "Δimu_reply_miss=%d, Δimu_reply_over10=%d (hedef 10 ms aşımı, bilgi)" % (
                  d["imu_reply_miss"], d["imu_reply_over10"]),
              d["imu_reply_miss"] == 0)
    # Her istek kartta cevaplandı mı (rapor 092): host'taki zaman aşımı, cevap karttan çıktıysa USB /
    # ST-LINK VCP'de kaybolan satırdır (kartın dışında; deadline tanımına girmez). Kartın OK cevap sayısı
    # = ilk GET_IMU (adım 3) + poller'ın istekleri; TX_UART_STALL artmadıysa her satır UART'tan tam çıktı.
    board_ok = d["cmd_imu_ok"] - (1 if first is not None else 0)
    lost_after_board = s["timeouts"] if board_ok == s["requests"] and d["tx_uart_stall"] == 0 else None
    res.check("her istek cevaplandı (kart)", "kart her isteğe OK cevap verdi, hata yok; host zaman aşımı yalnız kartın dışında",
              "kart OK %d / istek %d, host zaman aşımı %d%s, Δtx_uart_stall %d, hata %s" % (
                  board_ok, s["requests"], s["timeouts"],
                  " (cevap karttan çıktı, USB / VCP'de kayboldu)" if lost_after_board else "",
                  d["tx_uart_stall"], s["errors"]),
              s["ok"] > 0 and not s["errors"] and (s["timeouts"] == 0 or lost_after_board is not None))
    rx_err = {k: v for k, v in d.items() if v and (
        k.startswith(("uart_rx", "line_", "frame_", "ip_", "udp_", "rx_", "pkt_", "cb_crit_sb", "dgram_"))
        or k in ("cmd_imu_refused", "cmd_invalid", "ctrlq_full", "pool_crit_empty"))
        and not k.startswith(("rx_fcs", "pkt_ok_")) and k not in ("rx_bytes", "rx_lines_total", "rx_frames")}
    res.info("kart: giriş yolu ret / kayıp sayaçları (sıfır olmayanlar)", rx_err or "yok")
    res.info("kart: GET_IMU dışındaki komutlar", "Δcmd_conflict=%d, Δcmd_replay=%d (GET_IMU tekrar penceresine girmez)" % (
        d["cmd_conflict"], d["cmd_replay"]))
    res.note("Deadline yalnız kart zamanıyla: M1 (istek UDP callback'ine geldi) -> M6 (cevabın son biti "
             "UART'tan çıktı, TX bitti kesmesi). Windows / USB hiçbir ölçüme girmez.")
    return res
