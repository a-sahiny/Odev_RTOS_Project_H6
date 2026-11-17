"""Komut satırı. Örnekler (Host klasöründen):

    python vnet.py ports
    python vnet.py --port COM5 hello
    python vnet.py --port COM5 monitor
    python vnet.py --port COM5 start --rate 20 --seconds 5
    python vnet.py --port COM5 stats
    python vnet.py --port COM5 imu --seconds 5
    python vnet.py --port COM5 test T01 T02 T05
    python vnet.py --sim test all --minutes 2
    python vnet.py gui
"""

import argparse
import json
import os
import sys
import time

from . import accounting as acc
from . import ethframe as eth
from . import link as linkmod
from . import proto
from .hoststack import Host, HostError
from . import FROZEN, HOST_DIR
from .stats_ids import COUNTER_NAMES


def _open(args, log=None):
    if args.sim:
        from .sim_board import SimBoard
        lk = linkmod.SimLink(SimBoard(seed=args.seed))
    else:
        port = args.port or linkmod.guess_stlink_port()
        if not port:
            raise SystemExit("Port bulunamadı. 'python vnet.py ports' ile bakın, --port COMx verin ya da --sim kullanın.")
        lk = linkmod.SerialLink(port, args.baud)
    flag = {"none": None, "0": 0, "1": 1}[args.fcs]
    return Host(lk, fcs_flag=flag, log=log)


def _printer(verbose):
    def log(direction, text):
        if verbose or direction in ("rx!", "info"):
            print("%-4s %s" % (direction, text))
    return log


def cmd_ports(args):
    ports = linkmod.list_ports()
    if not ports:
        print("Seri port yok (pyserial yüklü mü?).")
    for dev, desc in ports:
        print("%-8s %s%s" % (dev, desc, "   <- ST-LINK" if "STLink" in desc or "STMicro" in desc else ""))


def cmd_monitor(args, h):
    def show(kind, data):
        if kind == "report":
            print(time.strftime("%H:%M:%S"), "JSON", json.dumps({k: v for k, v in data.items() if not k.startswith("_")},
                                                                ensure_ascii=False))
        elif kind == "telem":
            print(time.strftime("%H:%M:%S"), "TELEM seq %5d idx %7d tick %8d a=%s g=%s" % (
                data.seq, data.index, data.tick, data.accel, data.gyro))
        elif kind == "response":
            print(time.strftime("%H:%M:%S"), "CEVAP REQ_ID %d %s %s" % (data.req_id, proto.TYPE_NAMES.get(data.type),
                                                                       data.status_name))
        elif kind == "frame_err":
            print(time.strftime("%H:%M:%S"), "HATA", data[0], data[1][:100])
    h.listeners.append(show)
    print("Dinleniyor (Ctrl+C ile çık). Kartın ARP isteklerine cevap veriliyor.")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


def cmd_stats(args, h):
    c = h.get_counters()
    print("tick %d ms" % c["_tick"])
    for i, n in enumerate(COUNTER_NAMES):
        if c[n] or args.all:
            print("%3d %-22s %10d" % (i, n, c[n]))
    env = h.get_env()
    if env:
        print("ENV kanalları:", env)


def cmd_imu(args, h):
    """START, sonra GET_IMU'yu periyotla sorar; her 10. örneği ve sonunda kart zamanıyla deadline özetini yazar."""
    from .hoststack import ImuPoller
    r = h.start()
    print("START: %s, STATE %s" % (r.status_name, proto.state_text(r.state)))
    if not r.ok:
        print("START reddedildi (%s): GET_IMU E_STATE döner." % r.status_name)
        return 2
    count = [0]

    def show(imu):
        count[0] += 1
        if count[0] % 10 == 1:
            print("idx %7d  roll %8.2f  pitch %7.2f  yaw %8.2f  %s  kart M1->M6 (önceki) %6.2f ms  LAT_US %4d" % (
                imu.index, imu.roll, imu.pitch, imu.yaw, "bias hazır" if imu.bias_ready else "bias ölçülüyor",
                imu.prev_us / 1000.0, imu.lat_us))
    poller = ImuPoller(h, period_ms=args.period, on_sample=show)
    poller.start()
    try:
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        pass
    poller.stop()
    print(json.dumps(poller.stats.summary(), ensure_ascii=False))
    c = h.get_counters()
    print("kart: imu_reply_over10 %d, imu_reply_miss %d (M1 -> M6, deadline 12 ms)" % (
        c["imu_reply_over10"], c["imu_reply_miss"]))
    return 0


def cmd_test(args, h):
    import tests
    ids = list(tests.TESTS) if [t.upper() for t in args.ids] == ["ALL"] else [t.upper() for t in args.ids]
    out_dir = args.out or os.path.join(HOST_DIR, "logs")
    failed = 0
    for tid in ids:
        if tid == "T07" and "ALL" in [t.upper() for t in args.ids] and args.minutes is None:
            print("T07 atlandı (uzun test): --minutes ile ya da tek başına çalıştırın.")
            continue
        run = tests.get(tid)
        kw = {}
        if args.minutes is not None:
            kw["minutes"] = args.minutes
            kw["slice_s"] = max(10.0, args.minutes * 60.0 / 6.0)
        if args.seconds is not None:
            kw["seconds"] = args.seconds
        print("\n>>> %s %s" % (tid, tests.TESTS[tid][1]))
        t0 = time.perf_counter()
        try:
            res = run(h, **kw)
        except HostError as e:
            print("%s HATA: %s" % (tid, e))
            failed += 1
            continue
        res.meta["seconds"] = round(time.perf_counter() - t0, 1)
        print(res.text())
        print("kayıt:", res.save(out_dir) + ".json/.csv")
        failed += 0 if res.passed else 1
    return 1 if failed else 0


def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    if not FROZEN:
        sys.path.insert(0, HOST_DIR)        # 'tests' paketi; exe'de paketin içinde gelir
    p = argparse.ArgumentParser(prog="vnet", description="STM32F746 telemetri istasyonu host aracı")
    p.add_argument("--port", help="seri port (ör. COM5); verilmezse ST-LINK aranır")
    p.add_argument("--baud", type=int, default=625000, help="UART1 hızı (rapor 086: 625000)")
    p.add_argument("--sim", action="store_true", help="kart yerine Python simülatörü")
    p.add_argument("--seed", type=int, default=1, help="simülatör seed'i")
    p.add_argument("--fcs", choices=("none", "0", "1"), default="1", help="host->kart FRAME biçimi")
    p.add_argument("-v", "--verbose", action="store_true", help="her satırı yaz")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ports", help="seri portları listele")
    sub.add_parser("gui", help="grafik arayüz")
    sub.add_parser("monitor", help="karttan geleni canlı göster")
    sub.add_parser("hello")
    sub.add_parser("state")
    sp = sub.add_parser("ping")
    sp.add_argument("-n", type=int, default=4)
    sp = sub.add_parser("start", help="START (+ isteğe bağlı süre boyunca telemetri göster)")
    sp.add_argument("--rate", type=int)
    sp.add_argument("--seconds", type=float, default=0)
    sub.add_parser("stop")
    sp = sub.add_parser("rate")
    sp.add_argument("hz", type=int)
    sp = sub.add_parser("stats", help="GET_STATS sayfa 0..3")
    sp.add_argument("--all", action="store_true", help="sıfır sayaçları da yaz")
    sp = sub.add_parser("hold")
    sp.add_argument("ms", type=int)
    sp = sub.add_parser("txdelay")
    sp.add_argument("ms", type=int)
    sp = sub.add_parser("env")
    sp.add_argument("values", type=int, nargs="+")
    sp = sub.add_parser("bulk")
    sp.add_argument("--seed", dest="bseed", type=lambda x: int(x, 0), default=0x12345678)
    sp.add_argument("--bytes", type=int, default=64)
    sp = sub.add_parser("cal")
    sp.add_argument("offsets", type=int, nargs=6)
    sp = sub.add_parser("imu", help="START + GET_IMU (roll / pitch / yaw), 20 ms periyotla")
    sp.add_argument("--seconds", type=float, default=5)
    sp.add_argument("--period", type=float, default=proto.IMU_PERIOD_MS, help="istek periyodu, ms (varsayılan 20)")
    sp = sub.add_parser("test", help="T01..T08 ya da all")
    sp.add_argument("ids", nargs="+")
    sp.add_argument("--minutes", type=float, help="T07 süresi (varsayılan 30)")
    sp.add_argument("--seconds", type=float, help="T04 seviye / T08 süresi (varsayılan 10)")
    sp.add_argument("--out", help="kayıt klasörü (varsayılan Host/logs)")
    args = p.parse_args(argv)

    if args.cmd == "ports":
        cmd_ports(args)
        return 0
    if args.cmd == "gui":
        from .gui import main as gui_main
        gui_main()
        return 0

    h = _open(args, _printer(args.verbose))
    try:
        if args.cmd == "test":
            return cmd_test(args, h)
        hello = h.connect()
        print("Kart: %s fw %s ep %s, MAC %s" % (hello.get("board"), hello.get("fw"), hello.get("ep"),
                                               eth.mac_str(h.board_mac)))
        if args.cmd == "hello":
            print(json.dumps({k: v for k, v in hello.items() if not k.startswith("_")}, ensure_ascii=False))
        elif args.cmd == "state":
            st = h.state()
            print(json.dumps({k: v for k, v in st.items() if not k.startswith("_")}, ensure_ascii=False) if st else
                  "cevap yok")
        elif args.cmd == "ping":
            for i in range(args.n):
                rtt = h.ping(seq=i + 1)
                print("ping %d: %s" % (i + 1, "%.1f ms" % rtt if rtt is not None else "zaman aşımı"))
        elif args.cmd == "start":
            if args.rate:
                print("SET_RATE:", h.set_rate(args.rate).status_name)
            r = h.start()
            print("START: %s, STATE %s, LAT_US %d, RTT %.1f ms" % (r.status_name, proto.state_text(r.state),
                                                                  r.lat_us, r.rtt_ms))
            if not r.ok:
                print("START reddedildi (%s): ölçüm başlamadı, telemetri beklenmiyor." % r.status_name)
                return 2
            if args.seconds:
                h.listeners.append(lambda k, d: k == "telem" and print(
                    "TELEM seq %5d idx %6d a=%s g=%s" % (d.seq, d.index, d.accel, d.gyro)))
                time.sleep(args.seconds)
                print("alınan %d, kayıp (SEQ boşluğu) %d, %.1f Hz" % (h.telem.received, h.telem.gaps,
                                                                      h.telem.rate_hz()))
        elif args.cmd == "stop":
            print("STOP:", h.stop().status_name)
        elif args.cmd == "rate":
            r = h.set_rate(args.hz)
            print("SET_RATE: %s, uygulanan %s Hz" % (r.status_name, r.applied_rate))
        elif args.cmd == "stats":
            cmd_stats(args, h)
        elif args.cmd == "hold":
            print(h.hold(args.ms))
        elif args.cmd == "txdelay":
            print(h.txdelay(args.ms))
        elif args.cmd == "env":
            h.send_env(args.values)
            print("ENV gönderildi")
        elif args.cmd == "bulk":
            h.send_bulk(args.bseed, args.bytes)
            print("BULK gönderildi")
        elif args.cmd == "cal":
            h.send_cal(args.offsets)
            print("CAL gönderildi")
        elif args.cmd == "monitor":
            cmd_monitor(args, h)
        elif args.cmd == "imu":
            return cmd_imu(args, h)
        time.sleep(0.2)
        return 0
    except HostError as e:
        print("HATA:", e)
        return 2
    finally:
        h.link.close()


if __name__ == "__main__":
    sys.exit(main())
