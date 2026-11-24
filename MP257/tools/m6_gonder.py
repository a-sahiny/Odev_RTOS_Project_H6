# M6 gonderme olcumu, PC'de calisir: kart /dev/rpmsg0'dan paketleri okur, PC Host'un decode'u ile denetler.
# Karta dosya yazmaz; M33'e yalniz '1' (gonder, saniyede bir yenilenir) ve '0' (dur) komutu gider. Kullanim: python m6_gonder.py [saniye]   (varsayilan 60)
import base64
import pathlib
import subprocess
import sys

KLASOR = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(KLASOR.parents[1] / "Host"))
from vnet_host import proto  # noqa: E402

KART = "root@192.168.7.1"
HZ = 100
CPU_MHZ = 400                       # k_cycle_get_32 frekansi

# Kartta calisan ek: m4_olc.py'nin sonuna eklenir
KART_EK = '''
import base64
import select

SAYAC = SAYAC + ("link_sent", "link_skipped", "link_drops", "link_rx", "link_unknown", "link_expired", "ipcc_busy", "ipcc_errors")
SURE = SURE + (("proc_time", "T4-T3 isleme"), ("send_time", "T5-T4 Linux verme"))
AYGIT = "/dev/rpmsg0"
KOMUT = []


def komut(fd, harf):
    """M33'e tek baytlik komut yollar, sayar."""
    os.write(fd, harf)
    KOMUT.append(harf)


def topla(fd, sinir, sessizlik, saat, yenile=False):
    """Sinir anina kadar gelen paketler; ardindan hat sessizlik saniye susana kadar kalanlar. yenile: saniyede bir gonder komutu."""
    out, sonraki = [], saat() + 1
    while saat() < sinir + 5:           # M33 susmazsa burada biter
        kalan = sinir - saat()
        if yenile and saat() >= sonraki:
            komut(fd, b"1")
            sonraki += 1
        bekle = min(kalan, sonraki - saat()) if yenile else kalan
        if select.select([fd], [], [], max(0.0, bekle) if kalan > 0 else sessizlik)[0]:
            out.append(os.read(fd, 512))
        elif kalan <= 0:
            break
    return out


def kart_main(oku, saat, sure, ham):
    """Olcer, sayac farklarini ve paketleri yazar; 0 = okundu."""
    sem, adres, bas = elf_oku(ham)
    eksik = [k for k in SAYAC + tuple(k for k, _ in SURE) if k not in sem]
    if eksik:
        print("DUR: programda yok: %s (M6 Adim 4 programi yuklu degil)" % ", ".join(eksik))
        return 2
    if b"".join(oku(adres + i).to_bytes(4, "little") for i in range(0, 16, 4)) != bas:
        print("DUR: calisan program karttaki dosyayla ayni degil")
        return 2
    fd = os.open(AYGIT, os.O_RDWR)
    try:
        komut(fd, b"0")
        topla(fd, saat(), 0.3, saat)    # onceden birikmis paketler atilir
        a = anlik(oku, sem)
        KOMUT.clear()
        print("%.0f saniye okunuyor..." % sure, flush=True)
        komut(fd, b"1")
        paket = topla(fd, saat() + sure, 0.0, saat, True)
        komut(fd, b"0")
        paket += topla(fd, saat(), 0.5, saat)
        b = anlik(oku, sem)
    finally:
        os.write(fd, b"0")              # okuyan kalmiyor: M33 dursun
        os.close(fd)
    print("SAYAC komut %d" % len(KOMUT))
    for k in SAYAC:
        print("SAYAC %s %d" % (k, (b[k] - a[k]) & 0xFFFFFFFF))
    for k, ad in SURE:
        n = (b[k][0] - a[k][0]) & 0xFFFFFFFF
        print("SURE %s %d %d %d %d" % (k, n, b[k][1], (b[k][3] - a[k][3]) // n if n else 0, b[k][2]))
    print("PAKET %s" % base64.b64encode(b"".join(len(p).to_bytes(2, "little") + p for p in paket)).decode())
    return 0
'''

KART_SON = '''
with open(ELF, "rb") as f:
    raise SystemExit(kart_main(oku_fiziksel, time.monotonic, %r, f.read()))
'''


def kart_kodu(sure):
    """m4_olc'nin kendi main'i calismasin diye __name__ degistirilir."""
    m4 = (KLASOR / "m4_olc.py").read_text(encoding="utf-8")
    return '__name__ = "m4"\n' + m4 + KART_EK + (KART_SON % sure if sure else "")


def ac(b64):
    """base64 -> paket listesi"""
    ham, i, out = base64.b64decode(b64), 0, []
    while i < len(ham):
        n = int.from_bytes(ham[i:i + 2], "little")
        out.append(ham[i + 2:i + 2 + n])
        i += 2 + n
    return out


def degerlendir(s, sureler, paketler, sure):
    """[(gecti, metin)]; kayip paketlerin SEQ'inden olculur."""
    iyi, seqler, hata = 0, [], ""
    for ham in paketler:
        try:
            p = proto.decode(ham)
        except ValueError as e:
            hata = hata or " (%s: %s)" % (e, ham.hex())
            continue
        iyi += p.type == proto.TELEM and p.flags == 0 and p.req_id == 0
        seqler.append(p.seq)
    bosluk = sum(((y - x) & 0xFFFF) != 1 for x, y in zip(seqler, seqler[1:]))
    n, bek = len(paketler), HZ * sure
    satir = [(abs(n - bek) <= bek * 0.02, "Linux'un aldigi paket %d (beklenen %.0f, pay %%2)" % (n, bek)),
             (n > 0 and iyi == n, "Host decode: %d / %d gecerli%s" % (iyi, n, hata)),
             (n > 1 and bosluk == 0, "SEQ boslugu %d" % bosluk),
             (s["link_sent"] == n, "M33'un gonderdigi %d, Linux'un aldigi %d" % (s["link_sent"], n)),
             (s["link_drops"] == 0, "tampon dolu kaybi %d" % s["link_drops"]),
             (s["link_unknown"] == 0 and s["link_rx"] == s["komut"],
              "M33'un aldigi komut %d (yollanan %d), taninmayan %d" % (s["link_rx"], s["komut"], s["link_unknown"])),
             (s["link_expired"] == 0, "yenilenmedigi icin durma %d" % s["link_expired"]),
             (s["ipcc_errors"] == 0, "IPCC hatasi %d, atlanan haber %d" % (s["ipcc_errors"], s["ipcc_busy"]))]
    for k, ad in (("bad_len", "yanlis uzunluk"), ("seq_gaps", "hat sira boslugu"),
                  ("rx_errors", "hat hatasi"), ("queue_drops", "kuyruk kaybi")):
        satir.append((s[k] == 0, "%s %d" % (ad, s[k])))
    for k, ad in (("rx_time", "T2-T1 ilk bayt -> 23. bayt"), ("wake_time", "T3-T2 kesme -> thread"),
                  ("proc_time", "T4-T3 isleme"), ("send_time", "T5-T4 Linux'a verme")):
        adet, az, ort, cok = sureler[k]
        us = [x / CPU_MHZ for x in (az, ort, cok)]
        satir.append((adet > 0 and 0 < az <= ort <= cok,
                      "%s: en az %.1f, ort %.1f, en cok %.1f us, jitter %.1f us (acilistan beri en az/en cok)"
                      % (ad, us[0], us[1], us[2], us[2] - us[0])))
    return satir


def main(sure):
    """Karti okutur ve degerlendirir; 0 = hepsi gecti."""
    ssh = subprocess.run(["ssh", KART, "python3 -"], input=kart_kodu(sure), stdout=subprocess.PIPE,
                         text=True, encoding="utf-8")
    sayac, sureler, paketler = {}, {}, None
    for s in ssh.stdout.splitlines():
        p = s.split()
        if s.startswith("SAYAC "):
            sayac[p[1]] = int(p[2])
        elif s.startswith("SURE "):
            sureler[p[1]] = tuple(int(x) for x in p[2:])
        elif s.startswith("PAKET"):
            paketler = ac(p[1] if len(p) > 1 else "")
        else:
            print(s)
    if ssh.returncode != 0 or paketler is None:
        print("SONUC: KALDI (kart tarafi tamamlanmadi, cikis kodu %d)" % ssh.returncode)
        return 1
    sonuc = degerlendir(sayac, sureler, paketler, sure)
    for gecti, metin in sonuc:
        print("%s  %s" % ("GECTI" if gecti else "KALDI", metin))
    print("bilgi: Linux istemedigi icin atilan %d" % sayac["link_skipped"])
    hepsi = all(g for g, _ in sonuc)
    print("SONUC: %s" % ("GECTI" if hepsi else "KALDI"))
    return 0 if hepsi else 1


if __name__ == "__main__":
    raise SystemExit(main(float(sys.argv[1]) if len(sys.argv) > 1 else 60.0))
