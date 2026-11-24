# M6 kanal kontrolu, PC'de calisir: M33 RPMsg kanalini acti mi, Linux /dev/rpmsg0'i olusturdu mu?
# Karta hicbir sey yazmaz. Kullanim: python m6_kanal.py
import pathlib
import subprocess

KART = "root@192.168.7.1"
KLASOR = pathlib.Path(__file__).resolve().parent

# Kartta calisan ek: m4_olc.py'nin sonuna eklenir
KART_EK = '''
import glob

SAY = ("link_state", "link_error", "link_rx", "ipcc_busy", "ipcc_errors", "msg_count")
DURUM = {0: "Linux bekleniyor", 1: "acik", 2: "kurulamadi"}


def kanal(oku, ham, kok):
    """[(gecti, metin)]; program yuklu degilse tek satir."""
    sem, adres, bas = elf_oku(ham)
    eksik = [k for k in SAY if k not in sem]
    if eksik:
        return [(False, "programda yok: %s (M6 Adim 3 programi yuklu degil)" % ", ".join(eksik))]
    if b"".join(oku(adres + i).to_bytes(4, "little") for i in range(0, 16, 4)) != bas:
        return [(False, "calisan program karttaki dosyayla ayni degil")]
    d = {k: oku(sem[k]) for k in SAY}
    hata = d["link_error"] - (1 << 32) if d["link_error"] >> 31 else d["link_error"]
    satir = [(d["link_state"] == 1 and hata == 0, "M33 kanali: %s, hata kodu %d" % (DURUM.get(d["link_state"], "?"), hata)),
             (d["ipcc_errors"] == 0, "IPCC hatasi %d, atlanan haber %d" % (d["ipcc_errors"], d["ipcc_busy"])),
             (d["msg_count"] > 0, "telemetri calisiyor: msg_count %d" % d["msg_count"])]
    aygit = glob.glob(os.path.join(kok, "sys/bus/rpmsg/devices/*.rpmsg-raw.*"))
    satir.append((len(aygit) == 1, "Linux kanali: %s" % (", ".join(os.path.basename(a) for a in aygit) or "yok")))
    surucu = [os.path.basename(os.path.realpath(os.path.join(a, "driver"))) for a in aygit if os.path.exists(os.path.join(a, "driver"))]
    satir.append((surucu == ["rpmsg_chrdev"], "surucu: %s" % (", ".join(surucu) or "bagli degil")))
    dev = glob.glob(os.path.join(kok, "dev/rpmsg[0-9]*"))
    satir.append((len(dev) == 1, "aygit dosyasi: %s" % (", ".join("/dev/" + os.path.basename(x) for x in dev) or "yok")))
    return satir
'''

KART_SON = '''
with open(ELF, "rb") as f:
    sonuc = kanal(oku_fiziksel, f.read(), "/")
for gecti, metin in sonuc:
    print("%s  %s" % ("GECTI" if gecti else "KALDI", metin))
print("SONUC: %s" % ("GECTI" if all(g for g, _ in sonuc) else "KALDI"))
'''


def kart_kodu():
    """m4_olc'nin kendi main'i calismasin diye __name__ degistirilir."""
    return '__name__ = "m4"\n' + (KLASOR / "m4_olc.py").read_text(encoding="utf-8") + KART_EK


if __name__ == "__main__":
    raise SystemExit(subprocess.run(["ssh", KART, "python3 -"], input=kart_kodu() + KART_SON, text=True,
                                    encoding="utf-8").returncode)
