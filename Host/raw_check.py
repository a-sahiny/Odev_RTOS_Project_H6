"""MP257 derlemesinin USART6 ham hattını PC'den sınar: 3,3 V USB-seri dönüştürücünün RX'i F746 D1'e (PC6),
GND GND'ye. Çalıştır:  python Host/raw_check.py COM7
Kart açılışta 100 Hz yollar. GEÇTİ/KALDI basar; çıkış kodu 0 = hepsi geçti."""

import struct
import sys
import time

BAUD = 460800
LEN = 23                        # 9 x i16, flags u8, prev_seq u16, prev_us u16 (big-endian)
FLAG_PREV = 0x04


def prev(buf, i):
    """i'deki mesajın (flags, prev_seq, prev_us) alanları."""
    return struct.unpack_from(">BHH", buf, i + 18)


def find_start(buf):
    """Mesajda başlık yok: 20 mesaj boyunca prev_seq'i birer artan ilk konum, yoksa None."""
    for i in range(LEN):
        if len(buf) < i + 20 * LEN:
            return None
        ok = True
        for k in range(19):
            fa, sa, _ = prev(buf, i + k * LEN)
            fb, sb, _ = prev(buf, i + (k + 1) * LEN)
            if not (fa & FLAG_PREV and fb & FLAG_PREV and (sb - sa) & 0xFFFF == 1):
                ok = False
                break
        if ok:
            return i
    return None


def evaluate(buf, seconds, rate=100):
    """[(ad, sonuç metni, geçti mi)] döndürür."""
    out = []
    start = find_start(buf)
    out.append(("mesaj sınırı bulundu", "%d bayt alındı" % len(buf), start is not None))
    if start is None:
        return out

    msgs = [prev(buf, i) for i in range(start, len(buf) - LEN + 1, LEN)]
    hz = len(msgs) / seconds
    gaps = sum(1 for a, b in zip(msgs, msgs[1:]) if (b[1] - a[1]) & 0xFFFF != 1)
    us = [m[2] for m in msgs]
    lo, mean, hi = min(us), sum(us) / len(us), max(us)

    out.append(("hız %d ± %d msg/s" % (rate, rate * 0.02), "%.1f" % hz, abs(hz - rate) <= rate * 0.02))
    out.append(("her mesaj 23 bayt, prev_seq farkı 1", "%d mesaj, %d bozulma" % (len(msgs), gaps), gaps == 0))
    out.append(("DRDY -> son bit 0,5..3 ms", "min %d, ort %.0f, max %d us, jitter (max-min) %d us" % (
        lo, mean, hi, hi - lo), lo >= 500 and hi <= 3000))
    return out


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    import serial
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    ser = serial.Serial(sys.argv[1], BAUD, timeout=0.05)
    ser.reset_input_buffer()
    buf, t0 = bytearray(), time.perf_counter()
    while time.perf_counter() - t0 < seconds:
        buf += ser.read(4096)
    took = time.perf_counter() - t0
    ser.close()

    res = evaluate(buf, took)
    for name, text, ok in res:
        print("%s  %s: %s" % ("GEÇTİ" if ok else "KALDI", name, text))
    return 0 if all(r[2] for r in res) else 1


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.exit(main())
