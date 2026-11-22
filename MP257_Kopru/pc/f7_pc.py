#!/usr/bin/env python3
"""PC aracı: MP257'deki f7hub'a bağlanır (USB ağı: 192.168.7.1). Yalnız standart kütüphane.

    python pc\\f7_pc.py durum                 anlık durum (F746, M33, hat sayaçları)
    python pc\\f7_pc.py izle                  canlı olaylar: telemetri, açı, M33, raporlar (Ctrl+C ile çık)
    python pc\\f7_pc.py start | stop          ölçümü başlat / durdur
    python pc\\f7_pc.py rate 20               telemetri hızı (1..50 Hz)
    python pc\\f7_pc.py stats 0               GET_STATS sayfası (0..3)
    python pc\\f7_pc.py limit 2500            M33 eğim alarm sınırı (0,01 derece)
    python pc\\f7_pc.py konsol HELLO          F746 konsol satırı (HELLO, STATE, HOLD n, TXDELAY n)
"""

import argparse
import json
import sys
import urllib.request


def post(base, body):
    req = urllib.request.Request(base + "/api/cmd", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


def watch(base, kinds):
    with urllib.request.urlopen(base + "/events", timeout=60) as r:
        kind = None
        for raw in r:
            line = raw.decode("utf-8", "replace").rstrip("\n")
            if line.startswith("event: "):
                kind = line[7:]
            elif line.startswith("data: ") and (not kinds or kind in kinds):
                data = json.loads(line[6:])
                if kind == "telem":
                    print("TM  seq=%5d acc=%s gyro=%s" % (data["seq"], data["acc"], data["gyro"]))
                elif kind == "imu":
                    print("AT  roll=%7.2f pitch=%7.2f yaw=%7.2f rtt=%.1f ms" % (
                        data["roll_cdeg"] / 100, data["pitch_cdeg"] / 100, data["yaw_cdeg"] / 100, data["rtt_ms"]))
                elif kind == "m33":
                    print("M33 %-8s tm_age=%s ms gaps=%s dt=%s..%s us" % (
                        data["mode_text"], data["tm_age_ms"], data["tm_gaps"], data["dt_min_us"], data["dt_max_us"]))
                else:
                    print("%-4s %s" % (kind, json.dumps(data, ensure_ascii=False)))


def main():
    ap = argparse.ArgumentParser(description="MP257 f7hub istemcisi")
    ap.add_argument("--board", default="192.168.7.1", help="kartin IP'si (USB agi: 192.168.7.1)")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("komut", choices=["durum", "izle", "start", "stop", "rate", "stats", "limit", "konsol"])
    ap.add_argument("deger", nargs="?", help="rate: Hz, stats: sayfa, limit: cdeg, konsol: satir")
    ap.add_argument("--tur", action="append", default=[], help="izle: yalniz bu olaylar (telem, imu, m33, report...)")
    a = ap.parse_args()
    base = "http://%s:%d" % (a.board, a.port)

    try:
        if a.komut == "durum":
            with urllib.request.urlopen(base + "/api/status", timeout=5) as r:
                print(json.dumps(json.loads(r.read()), indent=2, ensure_ascii=False))
        elif a.komut == "izle":
            watch(base, set(a.tur))
        elif a.komut in ("start", "stop"):
            print(json.dumps(post(base, {"cmd": a.komut}), ensure_ascii=False))
        elif a.komut == "rate":
            print(json.dumps(post(base, {"cmd": "rate", "hz": int(a.deger or 10)}), ensure_ascii=False))
        elif a.komut == "stats":
            print(json.dumps(post(base, {"cmd": "stats", "page": int(a.deger or 0)}), ensure_ascii=False))
        elif a.komut == "limit":
            print(json.dumps(post(base, {"cmd": "limit", "cdeg": int(a.deger or 3000)}), ensure_ascii=False))
        elif a.komut == "konsol":
            print(json.dumps(post(base, {"cmd": "console", "line": a.deger or "HELLO"}), ensure_ascii=False))
    except KeyboardInterrupt:
        return 0
    except OSError as e:
        print("baglanti hatasi: %s (kart acik mi, f7hub calisiyor mu, %s erisilebilir mi?)" % (e, base))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
