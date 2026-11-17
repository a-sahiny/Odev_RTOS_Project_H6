"""T01–T07 ortak altyapısı (tasarım 20.2).

Her test 'girdi -> beklenen -> gözlenen -> geçti/kaldı' satırları üretir; sabit seed,
trafik hızı, paket boyutu, süre ve firmware sürümü kaydedilir. Yapay enjeksiyon
(HOLD / TXDELAY) kullanan ölçümler 'inj' ile etiketlenir.
"""

import csv
import json
import os
import statistics
import time

from vnet_host import accounting as acc

ROW_HOOK = None                # arayüz için: fonksiyon(row); her satırda çağrılır


class TestResult:
    def __init__(self, tid, title):
        self.tid = tid
        self.title = title
        self.rows = []
        self.meta = {"test": tid, "title": title, "started": time.strftime("%Y-%m-%d %H:%M:%S")}
        self.notes = []
        self.on_row = None             # arayüz için: fonksiyon(row)

    def check(self, girdi, beklenen, gozlenen, ok, inj=""):
        row = {"girdi": str(girdi), "beklenen": str(beklenen), "gözlenen": str(gozlenen),
               "sonuç": "GEÇTİ" if ok else "KALDI", "inj": inj}
        self.rows.append(row)
        if self.on_row:
            self.on_row(row)
        if ROW_HOOK:
            ROW_HOOK(row)
        return ok

    def info(self, girdi, gozlenen, inj=""):
        """Ölçüm satırı: kabul kriteri yok, raporlanır."""
        row = {"girdi": str(girdi), "beklenen": "(ölçüm)", "gözlenen": str(gozlenen), "sonuç": "BİLGİ", "inj": inj}
        self.rows.append(row)
        if self.on_row:
            self.on_row(row)
        if ROW_HOOK:
            ROW_HOOK(row)

    def note(self, text):
        self.notes.append(text)

    @property
    def passed(self):
        return all(r["sonuç"] != "KALDI" for r in self.rows) and any(r["sonuç"] == "GEÇTİ" for r in self.rows)

    def save(self, out_dir):
        os.makedirs(out_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(out_dir, "%s_%s" % (self.tid, stamp))
        self.meta["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
        self.meta["passed"] = self.passed
        with open(base + ".json", "w", encoding="utf-8") as f:
            json.dump({"meta": self.meta, "rows": self.rows, "notes": self.notes}, f, ensure_ascii=False, indent=1)
        with open(base + ".csv", "w", encoding="utf-8-sig", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["girdi", "beklenen", "gözlenen", "sonuç", "inj"], delimiter=";")
            w.writeheader()
            w.writerows(self.rows)
        return base

    def text(self):
        out = ["== %s %s ==" % (self.tid, self.title)]
        for r in self.rows:
            out.append("[%s] %s -> beklenen: %s | gözlenen: %s%s" % (
                r["sonuç"], r["girdi"], r["beklenen"], r["gözlenen"], ("  {inj: %s}" % r["inj"]) if r["inj"] else ""))
        for n in self.notes:
            out.append("  not: " + n)
        out.append("SONUÇ: %s" % ("GEÇTİ" if self.passed else "KALDI"))
        return "\n".join(out)


def prepare(h, res, rate=None):
    """HELLO + ARP + STOP; firmware sürümü ve EPOCH kaydedilir."""
    hello = h.connect()
    res.meta["fw"] = hello.get("fw")
    res.meta["board"] = hello.get("board")
    res.meta["epoch"] = hello.get("ep")
    res.meta["link"] = getattr(h.link, "name", "?")
    res.meta["board_mac"] = h.board_mac.hex(":") if h.board_mac else None
    h.txdelay(0)
    h.stop()
    if rate is not None:
        h.set_rate(rate)
    h.drain()
    return hello


def counters(h):
    """Sayaç görüntüsü. Arka planda GET_IMU poller'ı varsa (GUI izlemesi) o anda kısa süre bekletilir
    (rapor 092): hat sakinleşir, görüntü ve host'un yazdığı bayt sayısı ('_host_tx') aynı anı görür."""
    with h.quiesce():
        for attempt in range(3):
            h.drain(quiet_s=0.15, timeout=2.0)
            timeouts0 = h.c["cmd_timeout"]
            c = h.get_counters()
            # Bir sayfanın cevabı gelmeyip aynı REQ_ID ile tekrarlandıysa (USB / VCP'de kesilen satır),
            # kart o GET_STATS'ı iki kez işledi: sayfalar arasında fazladan bir olay var, görüntü tek an
            # değil (rapor 092, kartta T06 L4 / L5 ±1). Görüntü yeniden alınır.
            if h.c["cmd_timeout"] == timeouts0:
                break
        c["_retakes"] = attempt
        c["_host_tx"] = h.link.tx_bytes
    return c


def check_equalities(res, d, label, host_tx_bytes=None):
    bad = [c for c in acc.equalities(d, host_tx_bytes=host_tx_bytes) if not c.ok]
    res.check("%s: tasarım 18.2 eşitlikleri (%d adet)" % (label, len(acc.equalities(d))),
              "hepsi tutar", "tutmayan yok" if not bad else "; ".join(
                  "%s %s (%d≠%d)" % (c.layer, c.text, c.lhs, c.rhs) for c in bad), not bad)
    return not bad


def stats_text(values, unit="ms"):
    if not values:
        return "örnek yok"
    v = sorted(values)
    p95 = v[min(len(v) - 1, int(0.95 * len(v)))]
    return "n=%d ort=%.1f%s p95=%.1f%s max=%.1f%s" % (len(v), statistics.mean(v), unit, p95, unit, v[-1], unit)
