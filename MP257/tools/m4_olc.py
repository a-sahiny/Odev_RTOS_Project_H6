# M4 olcumu: M33 sayaclarini kartta (Linux) iki kez okur, aradaki farki degerlendirir.
# Yalniz okur. Kullanim: python3 - [saniye]   (varsayilan 60)
import mmap
import os
import struct
import sys
import time

ELF = "/lib/firmware/mp257_m33.elf"
HZ = 100                            # F746 mesaj hizi
CPU_MHZ = 400                       # k_cycle_get_32 frekansi
SAYAC = ("msg_count", "seq_gaps", "bad_len", "queue_drops", "rx_errors")
SURE = (("rx_time", "T2-T1 ilk bayt -> 23. bayt"), ("wake_time", "T3-T2 kesme -> thread"))


def elf_oku(ham):
    """ELF32'den (sembol adresleri, ilk 16 baytin adresi ve kendisi)."""
    shoff = struct.unpack_from("<I", ham, 32)[0]
    phoff = struct.unpack_from("<I", ham, 28)[0]
    phn = struct.unpack_from("<H", ham, 44)[0]
    shn = struct.unpack_from("<H", ham, 48)[0]
    bolum = [struct.unpack_from("<10I", ham, shoff + 40 * i) for i in range(shn)]
    sem = {}
    for b in bolum:
        if b[1] != 2:               # SHT_SYMTAB
            continue
        ad0 = bolum[b[6]][4]
        for i in range(b[4], b[4] + b[5], 16):
            n, deger = struct.unpack_from("<II", ham, i)
            son = ham.index(b"\0", ad0 + n)
            sem[ham[ad0 + n:son].decode("ascii", "replace")] = deger
    ilk = min((p for p in (struct.unpack_from("<8I", ham, phoff + 32 * i) for i in range(phn))
               if p[0] == 1 and p[4] >= 16), key=lambda p: p[3])
    return sem, ilk[3], ham[ilk[1]:ilk[1] + 16]


def oku_fiziksel(adres):
    """Fiziksel adresten 32 bit okur."""
    fd = os.open("/dev/mem", os.O_RDONLY | os.O_SYNC)
    try:
        with mmap.mmap(fd, 4096, mmap.MAP_SHARED, mmap.PROT_READ, offset=adres & ~0xFFF) as m:
            with memoryview(m) as v, v.cast("I") as w:
                return w[(adres & 0xFFF) // 4]
    finally:
        os.close(fd)


def ozet(oku, adres):
    """Stat yapisini tutarli okur: (adet, en az, en cok, toplam)."""
    for _ in range(20):
        n = oku(adres)
        d = (oku(adres + 4), oku(adres + 8), oku(adres + 16) | (oku(adres + 20) << 32))
        if oku(adres) == n:         # okurken thread degistirmedi
            return (n,) + d
    raise RuntimeError("sayac surekli degisiyor")


def anlik(oku, sem):
    """Butun sayaclar ve sure ozetleri."""
    s = {a: oku(sem[a]) for a in SAYAC}
    s.update({a: ozet(oku, sem[a]) for a, _ in SURE})
    return s


def degerlendir(a, b, sure):
    """[(gecti, metin)] satirlari."""
    fark = {k: (b[k] - a[k]) & 0xFFFFFFFF for k in SAYAC}
    bek = HZ * sure
    satir = [(abs(fark["msg_count"] - bek) <= bek * 0.02,
              "mesaj %d (beklenen %.0f, pay %%2), %.2f mesaj/s" % (fark["msg_count"], bek, fark["msg_count"] / sure))]
    for k, ad in (("bad_len", "yanlis uzunluk"), ("seq_gaps", "sira boslugu"),
                  ("rx_errors", "hat hatasi"), ("queue_drops", "kuyruk kaybi")):
        satir.append((fark[k] == 0, "%s %d" % (ad, fark[k])))
    for k, ad in SURE:
        n = (b[k][0] - a[k][0]) & 0xFFFFFFFF
        if n == 0:
            satir.append((False, "%s: olcum yok" % ad))
            continue
        ort = (b[k][3] - a[k][3]) / n
        us = [x / CPU_MHZ for x in (b[k][1], ort, b[k][2])]
        satir.append((0 < b[k][1] <= ort <= b[k][2],
                      "%s: en az %.1f, ort %.1f, en cok %.1f us, jitter %.1f us (acilistan beri en az/en cok)"
                      % (ad, us[0], us[1], us[2], us[2] - us[0])))
    return satir


def main(oku, uyu, sure, ham):
    """Olcer ve sonucu yazar; 0 = hepsi gecti."""
    sem, adres, bas = elf_oku(ham)
    eksik = [k for k in SAYAC + tuple(k for k, _ in SURE) if k not in sem]
    if eksik:
        print("DUR: programda yok: %s" % ", ".join(eksik))
        return 2
    if b"".join(oku(adres + i).to_bytes(4, "little") for i in range(0, 16, 4)) != bas:
        print("DUR: calisan program karttaki dosyayla ayni degil")
        return 2
    a = anlik(oku, sem)
    t0 = time.monotonic()
    print("acilistan beri: %s" % "  ".join("%s=%d" % (k, a[k]) for k in SAYAC))
    print("%.0f saniye olculuyor..." % sure, flush=True)
    uyu(sure)
    b = anlik(oku, sem)
    sonuc = degerlendir(a, b, sure if uyu is not time.sleep else time.monotonic() - t0)
    for gecti, metin in sonuc:
        print("%s  %s" % ("GECTI" if gecti else "KALDI", metin))
    hepsi = all(g for g, _ in sonuc)
    print("SONUC: %s" % ("GECTI" if hepsi else "KALDI"))
    return 0 if hepsi else 1


if __name__ == "__main__":
    with open(ELF, "rb") as f:
        raise SystemExit(main(oku_fiziksel, time.sleep, float(sys.argv[1]) if len(sys.argv) > 1 else 60.0, f.read()))
