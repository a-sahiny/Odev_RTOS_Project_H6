# M6 on kontrolu, kartta (Linux) calisir: RPMsg icin gereken bellek, posta kutusu ve suruculer var mi?
# Yalniz okur. Kullanim: python3 -
import os
import struct

DT = "/sys/firmware/devicetree/base"
MODUL = "/lib/modules/" + os.uname().release if hasattr(os, "uname") else ""
SURUCU = "/sys/bus/platform/devices/40490000.mailbox/driver"
BAS, SON = 0x81200000, 0x81300000   # Zephyr'de kullanilacak paylasilan bellek
ALAN = ("ipc-shmem-1@81200000", "vdev0vring0@812f8000", "vdev0vring1@812f9000", "vdev0buffer@812fa000")
TAMPON = 2 * 8 * 512                # iki yon, 8 tampon, 512 bayt


def oku(*yol):
    """Device tree ozelligi; yoksa bos."""
    try:
        with open(os.path.join(*yol), "rb") as f:
            return f.read()
    except OSError:
        return b""


def sayilar(ham):
    return struct.unpack(">%dI" % (len(ham) // 4), ham[:len(ham) // 4 * 4])


def yazi(ham):
    return ham.rstrip(b"\0").decode("ascii", "replace")


def kontrol(dt, modul, surucu):
    """[(gecti, metin)]"""
    satir, sonraki, tutamac, boy = [], BAS, [], {}
    for ad in ALAN:
        d = sayilar(oku(dt, "reserved-memory", ad, "reg"))
        if len(d) != 4:
            satir.append((False, "%s yok" % ad))
            continue
        adres, boy[ad] = (d[0] << 32) | d[1], (d[2] << 32) | d[3]
        satir.append((adres == sonraki, "%s: 0x%08X, %d bayt" % (ad, adres, boy[ad])))
        sonraki = adres + boy[ad]
        tutamac += sayilar(oku(dt, "reserved-memory", ad, "phandle"))
    satir.append((sonraki == SON, "dort alan bitisik ve 0x%08X'de bitiyor (beklenen 0x%08X)" % (sonraki, SON)))
    satir.append((boy.get(ALAN[3], 0) >= TAMPON, "RPMsg tamponu %d bayt (gereken %d)" % (boy.get(ALAN[3], 0), TAMPON)))
    m33 = os.path.join(dt, "ahb@1", "m33@0")
    liste = sayilar(oku(m33, "memory-region"))
    satir.append((len(tutamac) == 4 and all(t in liste for t in tutamac), "dort alan da M33'un listesinde"))
    kutu = yazi(oku(m33, "mbox-names")).split("\0")
    satir.append((kutu[:2] == ["vq0", "vq1"], "M33 posta kutulari: %s" % ", ".join(kutu)))
    satir.append((os.path.exists(surucu), "IPCC1 surucusu bagli"))
    for ko, takma in (("virtio_rpmsg_bus", "virtio:d00000007v*"), ("rpmsg_char", "rpmsg:rpmsg-raw")):
        hepsi = oku(modul, "modules.dep").decode("ascii", "replace") + oku(modul, "modules.builtin").decode("ascii", "replace")
        ad = oku(modul, "modules.alias").decode("ascii", "replace")
        satir.append(("/%s.ko" % ko in hepsi and "alias %s %s" % (takma, ko) in ad, "Linux surucusu %s (%s)" % (ko, takma)))
    return satir


def main(dt=DT, modul=MODUL, surucu=SURUCU):
    print("kart: %s" % yazi(oku(dt, "model")))
    print("cekirdek: %s" % os.path.basename(modul))
    sonuc = kontrol(dt, modul, surucu)
    for gecti, metin in sonuc:
        print("%s  %s" % ("GECTI" if gecti else "KALDI", metin))
    hepsi = all(g for g, _ in sonuc)
    print("SONUC: %s" % ("GECTI" if hepsi else "KALDI"))
    return 0 if hepsi else 1


if __name__ == "__main__":
    raise SystemExit(main())
