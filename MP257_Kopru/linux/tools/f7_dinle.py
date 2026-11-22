#!/usr/bin/env python3
"""Kartta çalışır: F746 seri hattını servis olmadan birkaç saniye dinler (ilk bağlantı testi).

    python3 /home/root/f7kopru/tools/f7_dinle.py /dev/ttyACM0

HELLO yollar, gelen satırları ekrana yazar: {"type":"hello",...} görünüyorsa hat ve 1000000 baud doğru.
f7linkd çalışıyorsa önce durdur (seri portun tek sahibi olmalı): systemctl stop f7linkd
"""

import os
import select
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from f7linkd import open_serial  # noqa: E402
from f7common import proto  # noqa: E402


def main():
    tty = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    fd = open_serial(tty, proto.UART_BAUD)
    os.write(fd, b"HELLO\n")
    end = time.monotonic() + seconds
    buf = bytearray()
    counts = {"json": 0, "frame": 0, "diger": 0}
    while time.monotonic() < end:
        r, _, _ = select.select([fd], [], [], 0.2)
        if not r:
            continue
        buf += os.read(fd, 4096)
        while b"\n" in buf:
            i = buf.index(b"\n")
            line = bytes(buf[:i]).rstrip(b"\r")
            del buf[:i + 1]
            if line.startswith(b"{"):
                counts["json"] += 1
                print(line.decode("ascii", "replace"))
            elif line.startswith(b"FRAME 1 "):
                counts["frame"] += 1
                print("FRAME (%d hex karakter)" % (len(line) - 8))
            else:
                counts["diger"] += 1
                print("? %r" % line[:60])
    os.close(fd)
    print("ozet: %s" % counts)


if __name__ == "__main__":
    main()
