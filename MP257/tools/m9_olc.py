# M9 olcumu, PC'de calisir: zincirin her halkasinin suresi ayri ayri (en az, ortalama, en cok, jitter = en cok - en az).
# m7_olc.py'nin kosusu (telem_udp, TERM) ve denetimleri aynen; ek olarak F746'nin ve Linux'un sureleri.
# Karta dosya yazmaz. Kartta M9 telem_udp'si olmali. Kullanim: python m9_olc.py [saniye]   (varsayilan 60)
import pathlib
import re
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import m7_olc as m7  # noqa: E402
from vnet_host import proto  # noqa: E402  (yolu m7_olc ekler)

M33 = (("rx_time", "M33   T2-T1 23 baytin gelisi"), ("wake_time", "M33   T3-T2 thread uyanmasi"),
       ("proc_time", "M33   T4-T3 paketleme"), ("send_time", "M33   T5-T4 Linux'a verme"))


def f746_us(paketler):
    """F746 DRDY -> son bit, us: her mesaj bir oncekinin suresini tasir (prev_us)."""
    us = []
    for _, _, ham in paketler:
        try:
            t = proto.parse_telem(proto.decode(ham))
        except ValueError:
            continue                        # bozuk paketi m7.degerlendir zaten sayar
        if t.prev_valid:
            us.append(t.prev_us)
    return us


def main(sure):
    """0 = hepsi gecti."""
    r = m7.olc(sure, "TERM")
    if r is None:
        return 1
    s, sureler, ek, paketler = r
    sonuc = m7.degerlendir(s, sureler, ek, paketler, sure, "TERM")
    tablo = []
    us = f746_us(paketler)
    if us:
        tablo.append(("F746  DRDY -> son bit", min(us), sum(us) / len(us), max(us)))
    for k, ad in M33:
        _, az, ort, cok = sureler[k]
        tablo.append((ad, az / 400, ort / 400, cok / 400))   # 400 MHz cevrim
    lx = re.search(r"linux_us en_az ([\d.]+) ort ([\d.]+) en_cok ([\d.]+)", ek["program"])
    if lx:
        tablo.append(("Linux poll -> send bitti",) + tuple(float(x) for x in lx.groups()))
    sonuc += [(len(us) > 0, "F746 suresi %d mesajdan" % len(us)),
              (lx is not None and 0 < float(lx[1]) <= float(lx[2]) <= float(lx[3]), "Linux suresi olculdu (M9 telem_udp)")]
    print("%-30s %9s %9s %9s %9s  (us)" % ("", "en az", "ort", "en cok", "jitter"))
    for ad, az, ort, cok in tablo:
        print("%-30s %9.1f %9.1f %9.1f %9.1f" % (ad, az, ort, cok, cok - az))
    print("bilgi: M33 en az / en cok acilistan beri; olculmeyen: M33 T5 -> Linux poll (cekirdek) ve Linux -> PC (ortak saat yok)")
    for gecti, metin in sonuc:
        print("%s  %s" % ("GECTI" if gecti else "KALDI", metin))
    hepsi = all(g for g, _ in sonuc)
    print("SONUC: %s" % ("GECTI" if hepsi else "KALDI"))
    return 0 if hepsi else 1


if __name__ == "__main__":
    raise SystemExit(main(float(sys.argv[1]) if len(sys.argv) > 1 else 60.0))
