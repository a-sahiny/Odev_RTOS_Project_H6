# M7 olcumu, PC'de calisir: kartta telem_udp'yi baslatir, UDP paketlerini dinler, Host'un decode'u ile denetler.
# Karta dosya yazmaz. Kullanim: python m7_olc.py [saniye] [bitis]   (varsayilan 60 TERM; bitis: TERM, INT, KILL)
import pathlib
import socket
import subprocess
import sys
import threading
import time

KLASOR = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(KLASOR.parents[1] / "Host"))
from vnet_host import proto  # noqa: E402

KART = "192.168.7.1"
PROGRAM = "/root/telem_udp"
PORT = 5002                         # Host'un TELEM portu
HZ = 100

# Kartta calisan ek: m4_olc.py'nin sonuna eklenir
KART_EK = '''
import resource
import signal
import subprocess

SAYAC = SAYAC + ("link_sent", "link_skipped", "link_drops", "link_rx", "link_unknown", "link_expired", "ipcc_busy", "ipcc_errors")
SURE = SURE + (("proc_time", "T4-T3 isleme"), ("send_time", "T5-T4 Linux verme"))


def kart_main(oku, sure, bitis, pc, port, ham):
    """Programi calistirir; sayac farklarini, program satirini ve islemci suresini yazar."""
    sem, adres, bas = elf_oku(ham)
    eksik = [k for k in SAYAC if k not in sem]
    if eksik:
        print("DUR: programda yok: %%s (M7 programi yuklu degil)" %% ", ".join(eksik))
        return 2
    if b"".join(oku(adres + i).to_bytes(4, "little") for i in range(0, 16, 4)) != bas:
        print("DUR: calisan program karttaki dosyayla ayni degil")
        return 2
    a = anlik(oku, sem)
    p = subprocess.Popen(["%s", pc, str(port)], stderr=subprocess.PIPE, text=True)
    time.sleep(sure)
    p.send_signal(getattr(signal, "SIG" + bitis))
    satir = p.communicate(timeout=10)[1].strip()
    t0 = time.monotonic()
    kapali = None
    while time.monotonic() - t0 < 6:
        if not oku(sem["link_on"]):
            kapali = time.monotonic() - t0
            break
        time.sleep(0.02)
    time.sleep(0.5)
    b = anlik(oku, sem)
    r = resource.getrusage(resource.RUSAGE_CHILDREN)
    for k in SAYAC:
        print("SAYAC %%s %%d" %% (k, (b[k] - a[k]) & 0xFFFFFFFF))
    for k, ad in SURE:
        n = (b[k][0] - a[k][0]) & 0xFFFFFFFF
        print("SURE %%s %%d %%d %%d %%d" %% (k, n, b[k][1], (b[k][3] - a[k][3]) // n if n else 0, b[k][2]))
    print("KAPANMA %%s" %% ("%%.2f" %% kapali if kapali is not None else "olmadi"))
    print("ISLEMCI %%.3f" %% (r.ru_utime + r.ru_stime))
    print("CIKIS %%d" %% p.returncode)
    print("PROGRAM %%s" %% satir)
    return 0
''' % PROGRAM

KART_SON = '''
with open(ELF, "rb") as f:
    raise SystemExit(kart_main(oku_fiziksel, %r, %r, %r, %r, f.read()))
'''


def kart_kodu(*arg):
    """m4_olc'nin kendi main'i calismasin diye __name__ degistirilir."""
    m4 = (KLASOR / "m4_olc.py").read_text(encoding="utf-8")
    return '__name__ = "m4"\n' + m4 + KART_EK + (KART_SON % arg if arg else "")


def dinle(sock, dur, paketler):
    """dur kurulana kadar gelen (an, gonderen, veri) listesi."""
    while not dur.is_set():
        try:
            veri, kim = sock.recvfrom(2048)
        except socket.timeout:
            continue
        except ConnectionResetError:
            continue                    # Windows: onceki gonderime ICMP cevabi
        paketler.append((time.perf_counter(), kim[0], veri))   # monotonic Windows'ta 15,6 ms adimli


def degerlendir(s, sureler, ek, paketler, sure, bitis):
    """[(gecti, metin)]; kayip SEQ'ten olculur."""
    iyi, seqler, hata = 0, [], ""
    for _, _, ham in paketler:
        try:
            p = proto.decode(ham)
        except ValueError as e:
            hata = hata or " (%s: %s)" % (e, ham.hex())
            continue
        iyi += p.type == proto.TELEM and p.flags == 0 and p.req_id == 0
        seqler.append(p.seq)
    bosluk = sum(((y - x) & 0xFFFF) != 1 for x, y in zip(seqler, seqler[1:]))
    n = len(paketler)
    satir = [(abs(n - HZ * sure) <= HZ * sure * 0.02, "PC'ye gelen UDP paketi %d (beklenen %.0f, pay %%2)" % (n, HZ * sure)),
             (n > 0 and iyi == n, "CRC / Host decode: %d / %d gecerli%s" % (iyi, n, hata)),
             (n > 1 and bosluk == 0, "SEQ boslugu %d" % bosluk),
             (n > 0 and {k for _, k, _ in paketler} == {KART}, "gonderen adres: %s" % ", ".join(sorted({k for _, k, _ in paketler}) or ["yok"])),
             (s["link_drops"] == 0 and s["ipcc_errors"] == 0, "M33 tampon kaybi %d, IPCC hatasi %d" % (s["link_drops"], s["ipcc_errors"])),
             (s["link_unknown"] == 0, "M33'un tanimadigi komut %d" % s["link_unknown"])]
    for k, ad in (("bad_len", "yanlis uzunluk"), ("seq_gaps", "hat sira boslugu"), ("rx_errors", "hat hatasi"), ("queue_drops", "kuyruk kaybi")):
        satir.append((s[k] == 0, "%s %d" % (ad, s[k])))
    if bitis == "KILL":
        satir += [(ek["cikis"] == -9, "program olduruldu (cikis %d)" % ek["cikis"]),
                  (s["link_expired"] == 1, "M33 sure dolunca durdu: %d kez" % s["link_expired"]),
                  (ek["kapanma"] is not None and 1.5 <= ek["kapanma"] <= 3.3, "durma: program oldukten %s s sonra (en cok 3 s + okuma payi)" % ek["kapanma"])]
    else:
        satir += [(ek["cikis"] == 0, "program duzgun kapandi (cikis %d)" % ek["cikis"]),
                  (s["link_sent"] == n, "M33'un verdigi %d, PC'nin aldigi %d" % (s["link_sent"], n)),
                  (s["link_expired"] == 0, "sure dolmasi %d (beklenen 0)" % s["link_expired"]),
                  (ek["kapanma"] is not None and ek["kapanma"] < 0.5, "durma: program kapandiktan %s s sonra" % ek["kapanma"]),
                  (" yollanan %d " % n in ek["program"] and "udp_hatasi 0 yenileme_hatasi 0" in ek["program"], ek["program"])]
    yuzde = 100.0 * ek["islemci"] / sure
    satir.append((yuzde < 2.0, "programin islemci payi %%%.2f (%.3f s / %.0f s)" % (yuzde, ek["islemci"], sure)))
    for k, ad in (("rx_time", "T2-T1"), ("wake_time", "T3-T2"), ("proc_time", "T4-T3"), ("send_time", "T5-T4")):
        adet, az, ort, cok = sureler[k]
        satir.append((adet > 0 and 0 < az <= ort <= cok, "%s: en az %.1f, ort %.1f, en cok %.1f us (acilistan beri en az/en cok)" % (ad, az / 400, ort / 400, cok / 400)))
    if len(paketler) > 1:
        ara = [(y[0] - x[0]) * 1000 for x, y in zip(paketler, paketler[1:])]
        print("bilgi: PC'de paket araligi en az %.2f, ort %.2f, en cok %.2f ms" % (min(ara), sum(ara) / len(ara), max(ara)))
    return satir


def main(sure, bitis):
    """Dinler, karti calistirir, degerlendirir; 0 = hepsi gecti."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind(("0.0.0.0", PORT))
    sock.settimeout(0.2)
    yon = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    yon.connect((KART, PORT))           # yalniz PC'nin bu hattaki adresini ogrenmek icin
    pc = yon.getsockname()[0]
    yon.close()
    paketler, dur = [], threading.Event()
    t = threading.Thread(target=dinle, args=(sock, dur, paketler))
    t.start()
    print("PC %s:%d dinliyor, %.0f saniye, bitis %s" % (pc, PORT, sure, bitis), flush=True)
    try:
        ssh = subprocess.run(["ssh", "root@" + KART, "python3 -"], input=kart_kodu(sure, bitis, pc, PORT),
                             stdout=subprocess.PIPE, text=True, encoding="utf-8")
    finally:
        dur.set()
        t.join()
        sock.close()
    s, sureler, ek = {}, {}, {"program": ""}
    for x in ssh.stdout.splitlines():
        p = x.split()
        if x.startswith("SAYAC "):
            s[p[1]] = int(p[2])
        elif x.startswith("SURE "):
            sureler[p[1]] = tuple(int(y) for y in p[2:])
        elif x.startswith("KAPANMA"):
            ek["kapanma"] = None if p[1] == "olmadi" else float(p[1])
        elif x.startswith("ISLEMCI"):
            ek["islemci"] = float(p[1])
        elif x.startswith("CIKIS"):
            ek["cikis"] = int(p[1])
        elif x.startswith("PROGRAM"):
            ek["program"] = x[8:] + " "
        else:
            print(x)
    if ssh.returncode != 0 or "cikis" not in ek:
        print("SONUC: KALDI (kart tarafi tamamlanmadi, cikis kodu %d)" % ssh.returncode)
        return 1
    sonuc = degerlendir(s, sureler, ek, paketler, sure, bitis)
    for gecti, metin in sonuc:
        print("%s  %s" % ("GECTI" if gecti else "KALDI", metin))
    hepsi = all(g for g, _ in sonuc)
    print("SONUC: %s" % ("GECTI" if hepsi else "KALDI"))
    return 0 if hepsi else 1


if __name__ == "__main__":
    raise SystemExit(main(float(sys.argv[1]) if len(sys.argv) > 1 else 60.0, sys.argv[2].upper() if len(sys.argv) > 2 else "TERM"))
