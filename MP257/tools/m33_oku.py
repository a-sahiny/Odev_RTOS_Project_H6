# M3 teshisi: M33 sayaclarini ve USART6/GPIOF yazmaclarini kartta (Linux) okur.
# Yalniz okur, hicbir sey yazmaz. Adresler 24.11.2026 tarihli zephyr.elf icindir.
import mmap
import os

RX_ERRORS = 0x80A002A0
RX_COUNT = 0x80A002A4
TX_COUNT = 0x80A002A8
TX_DATA = 0x80A00840
RX_DATA = 0x80A00458
TX_DESEN = 0x150E0700               # tx_data'nin ilk 4 bayti; dogru bellegi okudugumuzun kaniti

RCC_USART6 = 0x44200790
GPIOF = 0x44290000
USART6 = 0x40220000


def oku_fiziksel(adres):
    """Fiziksel adresten 32 bit okur."""
    fd = os.open("/dev/mem", os.O_RDONLY | os.O_SYNC)
    try:
        with mmap.mmap(fd, 4096, mmap.MAP_SHARED, mmap.PROT_READ, offset=adres & ~0xFFF) as m:
            with memoryview(m) as v, v.cast("I") as w:
                return w[(adres & 0xFFF) // 4]
    finally:
        os.close(fd)


def teshis(hata, gelen, giden):
    """Uc sayactan olasi nedeni secer."""
    if hata > 0:
        return "HAT HATASI: donanim %d hata bildirdi" % hata
    if giden == 0 and gelen == 0:
        return "KESME GELMIYOR: hic bayt gonderilmemis"
    if gelen == 0:
        return "KABLO ya da PIN: baytlar gitti, hicbiri gelmedi"
    if gelen >= 1000:
        return "HIZ ya da ICERIK: hepsi geldi ama gec kaldi ya da farkli"
    return "BAYT KAYBI: giden %d, gelen %d" % (giden, gelen)


def oku_blok(oku, adres, n):
    """n bayti 32 bitlik okumalarla alir."""
    ham = b"".join(oku(adres + i).to_bytes(4, "little") for i in range(0, n, 4))
    return ham[:n]


def karsilastir(tx, rx):
    """Gelen baytlari gidenlerle karsilastirir; kaybin yerini yazar."""
    n = len(rx)
    if n == 0:
        return "hic bayt gelmedi"
    if rx == tx[:n]:
        return "gelen %d baytin hepsi yerinde dogru" % n
    for k in range(len(tx)):        # k. bayt kayip, kalani sirali mi?
        if n == len(tx) - 1 and rx == tx[:k] + tx[k + 1:]:
            return "yalniz %d. bayt (0x%02X) kayip, kalan %d bayt sirali ve dogru" % (k, tx[k], n)
    ilk = next((i for i in range(n) if rx[i] != tx[i]), n)
    return "ilk fark %d. baytta, toplam farkli %d" % (ilk, sum(1 for i in range(n) if rx[i] != tx[i]))


def rapor(oku):
    """Butun degerleri okur ve satir satir yazar."""
    tx0 = oku(TX_DATA)
    print("tx_data ilk 4 bayt : 0x%08X" % tx0)
    if tx0 != TX_DESEN:
        print("DUR: 0x%08X bekleniyordu; farkli program calisiyor olabilir" % TX_DESEN)
        return 1
    hata, gelen, giden = oku(RX_ERRORS), oku(RX_COUNT), oku(TX_COUNT)
    print("rx_data ilk 4 bayt : 0x%08X" % oku(RX_DATA))
    print("rx_errors=%d rx_count=%d tx_count=%d" % (hata, gelen, giden))
    tx = oku_blok(oku, TX_DATA, 1000)
    rx = oku_blok(oku, RX_DATA, 1000)[:min(gelen, 1000)]
    if hata == 0 and rx == tx:
        print("TESHIS             : SORUN YOK, son tur tam")
    else:
        print("TESHIS             : %s" % teshis(hata, gelen, giden))
    print("ICERIK             : %s" % karsilastir(tx, rx))

    saat = oku(RCC_USART6)
    print("USART6 saat        : 0x%08X %s" % (saat, "acik" if saat & 2 else "KAPALI"))
    if saat & 2:                    # saat kapaliyken USART6 okunmaz
        for ad, ofs in (("CR1", 0x00), ("CR3", 0x08), ("BRR", 0x0C), ("ISR", 0x1C)):
            print("USART6 %-12s: 0x%08X" % (ad, oku(USART6 + ofs)))

    mod, idr, afr = oku(GPIOF), oku(GPIOF + 0x10), oku(GPIOF + 0x24)
    for pin in (13, 14):
        print("PF%d mod=%d (2=ozel islev) islev=AF%d (3 olmali) seviye=%d"
              % (pin, (mod >> (2 * pin)) & 3, (afr >> (4 * (pin - 8))) & 15, (idr >> pin) & 1))
    print("BITTI")
    return 0


if __name__ == "__main__":
    raise SystemExit(rapor(oku_fiziksel))
