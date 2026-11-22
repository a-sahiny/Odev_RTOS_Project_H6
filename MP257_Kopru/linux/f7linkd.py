#!/usr/bin/env python3
"""f7linkd: F746'nın UART hattını (ST-LINK USB, /dev/f746) Linux'ta gerçek bir ağ arayüzüne (tap-f7) çevirir.

F746 tarafı hiç değişmez: o hâlâ "FRAME 1 <hex>" satırlarıyla Ethernet çerçevesi yollar ve bekler.
Bu servis seri portun TEK sahibidir:
    F746 -> FRAME satırı -> FCS kontrolü -> tap-f7 (Linux çekirdeği ARP/ICMP/UDP'yi kendisi yapar)
    tap-f7 -> çerçeve -> 60 B dolgu + FCS -> FRAME satırı -> F746
    F746 -> {json} raporu -> UDP 127.0.0.1:7101 (f7hub dinler)
    UDP 127.0.0.1:7100 <- konsol komutu (HELLO, STATE, HOLD n, TXDELAY n) -> F746
Yalnız Python standart kütüphanesi; root olarak çalışır (TAP açmak ve ip komutları için).
"""

import argparse
import array
import errno
import fcntl
import json
import os
import re
import select
import socket
import struct
import subprocess
import sys
import termios
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from f7common import proto  # noqa: E402

# Linux ioctl sayıları (aarch64; asm-generic). termios2 = 4 x u32 + c_line + 19 c_cc + 2 x u32 hız = 44 B.
TCGETS2 = 0x802C542A
TCSETS2 = 0x402C542B
BOTHER = 0o010000
CBAUD = 0o010017
CIBAUD = CBAUD << 16
TUNSETIFF = 0x400454CA
IFF_TAP = 0x0002
IFF_NO_PI = 0x1000

COMMAND_RE = re.compile(rb"^(HELLO|STATE|HOLD [0-9]{1,5}|TXDELAY [0-9]{1,4})$")
TX_LIMIT = 16384            # seri TX kuyruğu: ~0,16 s hat süresi (1 Mbaud); dolarsa çerçeve atılır (F746'da akış kontrolü yok)


def log(msg):
    print(time.strftime("%H:%M:%S ") + msg, flush=True)


def open_serial(path, baud):
    """Seri portu ham modda açar; hızı (1000000, 625000 gibi) termios2 (BOTHER) ile tam olarak ayarlar."""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attr = termios.tcgetattr(fd)
    attr[0] = 0                                         # iflag: dönüşüm yok, XON/XOFF yok
    attr[1] = 0                                         # oflag: \n -> \r\n çevirme yok
    attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attr[3] = 0                                         # lflag: echo yok, satır düzenleme yok
    attr[6][termios.VMIN] = 0
    attr[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attr)

    buf = array.array("B", bytes(44))
    fcntl.ioctl(fd, TCGETS2, buf, True)
    iflag, oflag, cflag, lflag = struct.unpack_from("IIII", buf, 0)
    cflag = (cflag & ~(CBAUD | CIBAUD)) | BOTHER | (BOTHER << 16)
    struct.pack_into("IIII", buf, 0, iflag, oflag, cflag, lflag)
    struct.pack_into("II", buf, 36, baud, baud)          # c_ispeed, c_ospeed
    fcntl.ioctl(fd, TCSETS2, buf)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def open_tap(name, ip_cidr, mtu):
    """TAP arayüzü açar ve ayarlar: MAC, MTU (F746 çerçeve sınırı), IPv4, IPv6 kapalı."""
    fd = os.open("/dev/net/tun", os.O_RDWR | os.O_NONBLOCK)
    ifr = struct.pack("16sH22x", name.encode(), IFF_TAP | IFF_NO_PI)
    fcntl.ioctl(fd, TUNSETIFF, ifr)
    cmds = [
        ["ip", "link", "set", name, "address", proto.HOST_MAC],
        ["ip", "link", "set", name, "mtu", str(mtu)],
        ["sysctl", "-q", "-w", "net.ipv6.conf.%s.disable_ipv6=1" % name],   # seri hattı IPv6 gürültüsüyle doldurma
        ["ip", "addr", "flush", "dev", name],
        ["ip", "addr", "add", ip_cidr, "dev", name],
        ["ip", "link", "set", name, "up"],
    ]
    for cmd in cmds:
        subprocess.run(cmd, check=True)
    return fd


class Linkd:
    def __init__(self, args):
        self.args = args
        self.ser = None
        self.tap = open_tap(args.tap, args.ip, args.mtu)
        self.ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctl.bind(("127.0.0.1", args.ctl_port))
        self.out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_buf = bytearray()
        self.tx_buf = bytearray()
        self.c = dict(rx_lines=0, rx_frames=0, rx_json=0, rx_bad=0, tx_frames=0, tx_drop=0,
                      tx_long=0, cmds=0, cmd_bad=0, serial_reopen=0)
        self.last_json = 0.0

    # ------------------------------------------------------------ seri port
    def serial_open(self):
        try:
            self.ser = open_serial(self.args.tty, self.args.baud)
        except OSError as e:
            self.ser = None
            return "%s: %s" % (self.args.tty, e.strerror)
        self.rx_buf.clear()
        self.tx_buf.clear()
        self.queue_line(b"HELLO\n")                     # F746 kim olduğunu söylesin (hello raporu)
        log("seri port acik: %s @ %d" % (self.args.tty, self.args.baud))
        return None

    def serial_close(self, why):
        log("seri port kapandi: %s" % why)
        try:
            os.close(self.ser)
        except OSError:
            pass
        self.ser = None
        self.c["serial_reopen"] += 1

    def queue_line(self, line: bytes, droppable=False):
        """Satırı bütün olarak TX kuyruğuna ekler (satırlar asla iç içe geçmez: tek thread)."""
        if self.ser is None:
            return False
        if droppable and len(self.tx_buf) + len(line) > TX_LIMIT:
            self.c["tx_drop"] += 1
            return False
        self.tx_buf += line
        return True

    # ------------------------------------------------------------ F746 -> Linux
    def on_serial_data(self, data):
        self.rx_buf += data
        while True:
            i = self.rx_buf.find(b"\n")
            if i < 0:
                break
            line = bytes(self.rx_buf[:i]).rstrip(b"\r")
            del self.rx_buf[:i + 1]
            self.on_line(line)
        if len(self.rx_buf) > 2 * proto.LINE_MAX:
            self.rx_buf.clear()                         # satırsız çöp
            self.c["rx_bad"] += 1

    def on_line(self, line):
        self.c["rx_lines"] += 1
        if line.startswith(b"{"):
            self.c["rx_json"] += 1
            self.publish(line)
            return
        try:
            frame = proto.line_to_frame(line)
        except proto.PacketError:
            self.c["rx_bad"] += 1
            return
        self.c["rx_frames"] += 1
        try:
            os.write(self.tap, frame)
        except OSError:
            self.c["rx_bad"] += 1

    def publish(self, payload: bytes):
        try:
            self.out.sendto(payload, ("127.0.0.1", self.args.json_port))
        except OSError:
            pass                                        # hub çalışmıyor olabilir

    # ------------------------------------------------------------ Linux -> F746
    def on_tap_frame(self, frame):
        try:
            line = proto.frame_to_line(frame)
        except proto.PacketError:
            self.c["tx_long"] += 1                      # MTU doğruysa olmaz
            return
        if self.queue_line(line, droppable=True):
            self.c["tx_frames"] += 1

    def on_ctl(self, data, addr):
        cmd = data.strip()
        if COMMAND_RE.match(cmd) and self.queue_line(cmd + b"\n"):
            self.c["cmds"] += 1
            reply = b"ok"
        else:
            self.c["cmd_bad"] += 1
            reply = b"err"
        self.ctl.sendto(reply, addr)

    # ------------------------------------------------------------ döngü
    def status_json(self):
        obj = {"type": "linkd", "serial": self.ser is not None, "tty": self.args.tty, "tx_queue": len(self.tx_buf)}
        obj.update(self.c)
        return json.dumps(obj, separators=(",", ":")).encode()

    def run(self):
        next_try = 0.0
        while True:
            now = time.monotonic()
            if self.ser is None and now >= next_try:
                err = self.serial_open()
                if err:
                    next_try = now + 2.0                # kablo takılınca kendiliğinden bağlanır
            if now - self.last_json >= 1.0:
                self.last_json = now
                self.publish(self.status_json())

            rlist = [self.tap, self.ctl]
            wlist = []
            if self.ser is not None:
                rlist.append(self.ser)
                if self.tx_buf:
                    wlist.append(self.ser)
            r, w, _ = select.select(rlist, wlist, [], 0.2)

            if self.ser is not None and self.ser in r:
                try:
                    data = os.read(self.ser, 4096)
                    if not data:
                        raise OSError(errno.EIO, "EOF")
                    self.on_serial_data(data)
                except OSError as e:
                    if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                        self.serial_close(e.strerror or str(e))
            if self.ser is not None and self.ser in w:
                try:
                    n = os.write(self.ser, self.tx_buf[:4096])
                    del self.tx_buf[:n]
                except OSError as e:
                    if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                        self.serial_close(e.strerror or str(e))
            if self.tap in r:
                while True:
                    try:
                        frame = os.read(self.tap, 2048)
                    except BlockingIOError:
                        break
                    self.on_tap_frame(frame)
            if self.ctl in r:
                data, addr = self.ctl.recvfrom(256)
                self.on_ctl(data, addr)


def main():
    ap = argparse.ArgumentParser(description="F746 UART <-> tap-f7 koprusu")
    ap.add_argument("--tty", default="/dev/f746", help="F746 ST-LINK seri portu (udev kurali /dev/f746 yapar)")
    ap.add_argument("--baud", type=int, default=proto.UART_BAUD)
    ap.add_argument("--tap", default="tap-f7")
    ap.add_argument("--ip", default=proto.HOST_IP + "/24")
    ap.add_argument("--mtu", type=int, default=proto.FRAME_MAX - 14)
    ap.add_argument("--ctl-port", type=int, default=7100)
    ap.add_argument("--json-port", type=int, default=7101)
    args = ap.parse_args()
    Linkd(args).run()


if __name__ == "__main__":
    main()
