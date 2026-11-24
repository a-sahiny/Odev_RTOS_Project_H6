# M5 olcumu, PC'de calisir: kartta sayaclari ve son paketi okutur, paketleri Host'un decode'u ile denetler.
# Karta hicbir sey yazmaz. Kullanim: python m5_olc.py [saniye]   (varsayilan 60)
import pathlib
import subprocess
import sys

KLASOR = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(KLASOR.parents[1] / "Host"))
from vnet_host import proto  # noqa: E402

KART = "root@192.168.7.1"
ORNEK = 20                          # olcum boyunca okunan paket sayisi

# Kartta calisan ek: m4_olc.py'nin sonuna eklenir
KART_EK = '''
SURE = SURE + (("proc_time", "T4-T3 isleme (sira denetimi + paket)"),)
ORNEK = %d


def paket_oku(oku, sem):
    """(msg_count, 33 bayt); okurken thread degistirdiyse yeniden okur."""
    adres, kay = sem["last_packet"] & ~3, sem["last_packet"] & 3
    def bir():
        ham = b"".join(oku(adres + i).to_bytes(4, "little") for i in range(0, 36 + (kay > 0) * 4, 4))
        return ham[kay:kay + 33]
    for _ in range(20):
        n = oku(sem["msg_count"])
        p = bir()
        if bir() == p and oku(sem["msg_count"]) == n:
            return n, p
    raise RuntimeError("paket surekli degisiyor")


def uyu_ve_oku(oku, sem, bekle, saat):
    """Sureyi ORNEK parcaya boler, her parcada bir paket yazar."""
    def uyu(sure):
        t0 = saat()
        for i in range(1, ORNEK + 1):
            bekle(max(0.0, t0 + sure * i / ORNEK - saat()))
            n, p = paket_oku(oku, sem)
            print("PAKET %%d %%s" %% (n, p.hex()), flush=True)
    return uyu


def kart_main(oku, bekle, saat, sure, ham):
    sem = elf_oku(ham)[0]
    if "last_packet" not in sem:
        print("DUR: programda yok: last_packet (M5 programi yuklu degil)")
        return 2
    return main(oku, uyu_ve_oku(oku, sem, bekle, saat), sure, ham)
''' % ORNEK

KART_SON = '''
with open(ELF, "rb") as f:
    raise SystemExit(kart_main(oku_fiziksel, time.sleep, time.monotonic, %r, f.read()))
'''


def kart_kodu(sure):
    """Karta stdin'den verilen program; m4_olc'nin kendi main'i calismasin diye __name__ degistirilir."""
    m4 = (KLASOR / "m4_olc.py").read_text(encoding="utf-8")
    return '__name__ = "m4"\n' + m4 + KART_EK + (KART_SON % sure if sure else "")


def paketleri_denetle(ornekler):
    """ornekler: [(msg_count, bayt)]; [(gecti, metin)] dondurur."""
    iyi, sira, kayma, son, hata = 0, 0, set(), None, ""
    for n, ham in ornekler:
        try:
            p = proto.decode(ham)
            t = proto.parse_telem(p)
        except ValueError as e:
            hata = hata or " (%s: %s)" % (e, ham.hex())
            continue
        iyi += p.type == proto.TELEM and p.flags == 0 and p.req_id == 0
        sira += p.seq == ((t.prev_seq + 1) & 0xFFFF if t.prev_valid else 0)
        kayma.add((p.seq - n) & 0xFFFF)
        son = (p, t)
    top = len(ornekler)
    satir = [(top == ORNEK, "%d paket okundu (beklenen %d)" % (top, ORNEK)),
             (top > 0 and iyi == top, "Host decode: %d / %d paket gecerli%s" % (iyi, top, hata)),
             (top > 0 and sira == top, "SEQ = prev_seq + 1: %d / %d" % (sira, top)),
             (len(kayma) == 1, "SEQ ile msg_count birlikte ilerliyor: %d farkli kayma (beklenen 1)" % len(kayma))]
    if son:
        print("son paket: SEQ=%d prev_seq=%d ivme=%s g" % (son[0].seq, son[1].prev_seq, son[1].accel_g))
    return satir


def main(sure):
    """Karti okutur, iki tarafin sonucunu birlestirir; 0 = hepsi gecti."""
    ssh = subprocess.Popen(["ssh", KART, "python3 -"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                           text=True, encoding="utf-8")
    ssh.stdin.write(kart_kodu(sure))
    ssh.stdin.close()
    ornekler = []
    for s in ssh.stdout:
        if s.startswith("PAKET "):
            _, n, h = s.split()
            ornekler.append((int(n), bytes.fromhex(h)))
        elif not s.startswith("SONUC"):
            print(s, end="", flush=True)
    kod = ssh.wait()
    if kod not in (0, 1) or not ornekler:
        print("SONUC: KALDI (kart tarafi tamamlanmadi, cikis kodu %d)" % kod)
        return 1
    sonuc = paketleri_denetle(ornekler)
    for gecti, metin in sonuc:
        print("%s  %s" % ("GECTI" if gecti else "KALDI", metin))
    hepsi = kod == 0 and all(g for g, _ in sonuc)
    print("SONUC: %s" % ("GECTI" if hepsi else "KALDI"))
    return 0 if hepsi else 1


if __name__ == "__main__":
    raise SystemExit(main(float(sys.argv[1]) if len(sys.argv) > 1 else 60.0))
