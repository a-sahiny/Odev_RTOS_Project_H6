# M8 olcumu, PC'de calisir: GUI'nin MP257 (UDP) sekmesiyle ayni kod (vnet_host/udp_in.py) ile kartta
# telem_udp'yi ssh ile baslatir, UDP'yi dinler, durdurur. Karta dosya yazmaz. Kullanim: python m8_olc.py [saniye]
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "Host"))
from vnet_host import udp_in  # noqa: E402

HZ = 100


def main(sure):
    """0 = hepsi gecti."""
    rx = udp_in.UdpTelem()
    ssh = udp_in.run_board(udp_in.pc_ip())
    t0 = time.monotonic()
    while rx.telem.received == 0 and ssh.poll() is None and time.monotonic() - t0 < 10:
        time.sleep(0.05)                # ssh baglantisi ve ilk paket
    if rx.telem.received == 0:
        ssh.terminate()
        print("SONUC: KALDI (paket gelmedi; telem_udp: %s)" % ssh.communicate()[0].strip())
        rx.close()
        return 1
    print("ilk paket %.1f s sonra geldi, %.0f saniye olculuyor..." % (time.monotonic() - t0, sure), flush=True)
    n0 = rx.telem.received
    time.sleep(sure)
    n = rx.telem.received - n0
    calisti = ssh.poll() is None
    ssh.terminate()                     # GUI'deki Durdur ile ayni: ssh kopar, telem_udp '0' yollar
    ssh.wait()
    time.sleep(1.0)                     # yoldaki son paketler
    kalan = rx.telem.received
    time.sleep(1.0)
    sonra = rx.telem.received - kalan
    rx.close()
    t = rx.telem
    satir = [(abs(n - HZ * sure) <= HZ * sure * 0.02, "%.0f saniyede paket %d (beklenen %.0f, pay %%2), %.2f Hz" % (sure, n, HZ * sure, t.rate_hz())),
             (t.gaps == 0 and t.dups == 0, "SEQ boslugu %d, tekrar %d" % (t.gaps, t.dups)),
             (rx.bad == 0, "bozuk paket %d" % rx.bad),
             (calisti, "telem_udp olcum boyunca calisti"),
             (sonra == 0, "durdurduktan 1-2 s sonra gelen paket %d (beklenen 0: kartta telem_udp kapandi)" % sonra)]
    print("bilgi: son paket SEQ %d, roll %+.2f, pitch %+.2f, yaw %+.2f derece" % (t.last.seq, t.last.roll, t.last.pitch, t.last.yaw))
    for gecti, metin in satir:
        print("%s  %s" % ("GECTI" if gecti else "KALDI", metin))
    hepsi = all(g for g, _ in satir)
    print("SONUC: %s" % ("GECTI" if hepsi else "KALDI"))
    return 0 if hepsi else 1


if __name__ == "__main__":
    raise SystemExit(main(float(sys.argv[1]) if len(sys.argv) > 1 else 60.0))
