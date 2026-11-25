"""Kaynak seçimi, anlık hız, ssh oturumu ve iki IMU sayfası (H1–H6).

Çalıştırma (Host klasöründen):  python -m unittest discover -s selftest -v
"""

import os
import socket
import sys
import time
import tkinter as tk
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from vnet_host import proto, source as src, udp_in  # noqa: E402
from vnet_host.hoststack import TelemetryStats  # noqa: E402

PORT = 15003


def packet(seq, roll=0, yaw=0, f746=600, m33=582, flags=0x07):
    raw = proto.payload_telem((0, 0, 8192), (0, 0, 0), (roll, 0, yaw), flags, (seq - 1) & 0xFFFF, f746)
    return proto.encode(proto.Packet(proto.TELEM, 0, seq, 0, raw + m33.to_bytes(2, "big")))


def direct(seq, roll=0.0):
    """F746'nın UART'tan yolladığı TELEM."""
    return proto.Telem(seq=seq, accel=(0, 0, 8192), gyro=(0, 0, 0), roll=roll, pitch=0.0, yaw=0.0, flags=0x07,
                       prev_seq=seq - 1, prev_us=1950, rx_time=time.perf_counter())


class Source(unittest.TestCase):
    def setUp(self):
        self.s = src.TelemSource(PORT)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.tx.close()
        self.s.close()
        time.sleep(0.3)

    def send(self, p):
        self.tx.sendto(p, ("127.0.0.1", PORT))
        time.sleep(0.15)

    def test_h1_linux_present(self):
        """MP257 akarken doğrudan TELEM gösterilmez; MP257 kesilince ALIVE_S sonra gösterilir."""
        self.send(packet(1, roll=500))
        self.s.on_direct(direct(10, 33.0))
        self.assertEqual((self.s.active(), self.s.latest.roll, self.s.ignored), (src.MP257, 5.0, 1))
        self.assertEqual(self.s.direct_stats.received, 0)
        time.sleep(src.ALIVE_S + 0.1)
        self.assertEqual(self.s.active(), src.NONE)
        self.s.on_direct(direct(11, 44.0))
        self.assertEqual((self.s.active(), self.s.latest.roll), (src.DIRECT, 44.0))
        self.assertIs(self.s.stats(), self.s.direct_stats)
        self.send(packet(2, roll=700))                      # Linux geri geldi
        self.assertEqual((self.s.active(), self.s.latest.roll), (src.MP257, 7.0))

    def test_h3_yaw_zero_shared(self):
        self.send(packet(1, yaw=9000))
        self.s.yaw_zero = self.s.latest.yaw
        self.assertEqual(self.s.shown_yaw(self.s.latest), 0.0)
        self.s.yaw_zero = -179.0
        self.assertEqual(self.s.shown_yaw(direct(1)), 179.0)

    def test_h5_loss_not_summed(self):
        """Kayıptan sonraki paketin toplamı hesaplanmaz: prev_us ve m33_us hangi örneğe ait bilinmez."""
        for seq in (1, 2, 4, 5):
            self.send(packet(seq))
        self.assertEqual((self.s.udp.whole.n, self.s.udp.telem.gaps), (2, 1))
        self.assertEqual(self.s.udp.whole.last, 600 + 582 - 563)


def lines_of(text, seq, chunk=480):
    """M33'ün link_send_line'ı: satırı datagramlara böler."""
    out = []
    for pos in range(0, len(text), chunk):
        part = text[pos:pos + chunk]
        flags = (1 if pos == 0 else 0) | (2 if pos + chunk >= len(text) else 0)
        out.append(b"L" + bytes([flags]) + ((seq + len(out)) & 0xFFFF).to_bytes(2, "big") + part)
    return out


class Lines(unittest.TestCase):
    """D3: satır datagramları."""

    def test_join_and_loss(self):
        j = udp_in.LineJoin()
        short, long_ = b"HELLO\n", b"F" * 528 + b"\n"
        got = [j.add(d) for d in lines_of(short, 65534) + lines_of(long_, 65535) + lines_of(short, 1)]
        self.assertEqual(got, [short, None, long_, short])              # sıra 65535 -> 0 taşar
        a, b = lines_of(long_, 2)
        self.assertEqual([j.add(a), j.add(lines_of(short, 4)[0])], [None, short])   # ikinci parça kayıp
        self.assertEqual((j.lost, j.add(b"X"), j.bad), (1, None, 1))
        self.assertEqual(j.add(lines_of(long_, 7)[1]), None)             # ilk parçası gelmeyen satır
        self.assertEqual(j.add(lines_of(short, 9)[0]), short)

    def test_source_to_link(self):
        """UDP 'port + 1'e gelen satır Link'in satırı olur."""
        from vnet_host.link import Link
        lk, got = Link(), []
        lk.on_line = lambda text, t: got.append(text)
        s = src.TelemSource(PORT)
        self.addCleanup(time.sleep, 0.3)
        self.addCleanup(s.close)
        s.on_line = lk.feed_line
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for d in lines_of(b'{"type":"state"}\n', 0) + lines_of(b"F" * 528 + b"\n", 1):
            tx.sendto(d, ("127.0.0.1", PORT + 1))
        time.sleep(0.3)
        tx.close()
        self.assertEqual(got, ['{"type":"state"}', "F" * 528])
        self.assertEqual((lk.linux_lines, lk.rx_lines), (2, 2))


class Route(unittest.TestCase):
    """E3: yazılan baytların yolu."""

    def test_route(self):
        from vnet_host.link import Link

        class Wire(Link):
            def __init__(self):
                super().__init__()
                self.serial = b""

            def _write(self, data):
                self.serial += data

        s = src.TelemSource(PORT)
        self.addCleanup(time.sleep, 0.3)
        self.addCleanup(s.close)
        board = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(board.close)
        board.bind(("127.0.0.1", 15103))
        board.settimeout(0.5)
        s.lines.target = ("127.0.0.1", 15103)
        lk = Wire()
        lk.route = s.send_command
        lk.write(b"HELLO\n")                                # Linux yok: UART
        self.assertEqual((lk.serial, lk.linux_tx), (b"HELLO\n", 0))
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(tx.close)
        tx.sendto(packet(1), ("127.0.0.1", PORT))
        time.sleep(0.15)
        data = bytes(i % 251 for i in range(1000))
        lk.write(data)
        got = b""
        sizes = []
        while len(got) < len(data):
            part = board.recv(2048)
            sizes.append(len(part))
            got += part
        self.assertEqual((got, max(sizes), len(sizes)), (data, udp_in.CMD_MAX, 3))
        lk.write_fragmented(b"STATE\n", "bytes", gap_s=0)
        self.assertEqual(b"".join(board.recv(64) for _ in range(6)), b"STATE\n")
        self.assertEqual((lk.serial, lk.linux_tx, lk.tx_bytes), (b"HELLO\n", 1006, 1012))
        s.linux_gone()
        lk.write(b"X")
        self.assertEqual(lk.serial, b"HELLO\nX")

    def test_pace(self):
        """Yazım USART6'nın hızını geçmez: 20 KB en az (20000 - 2048) / 36000 s sürer, hepsi ulaşır."""
        board = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(board.close)
        board.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        board.bind(("127.0.0.1", 15104))
        board.settimeout(0.5)
        u = udp_in.UdpLines(PORT + 6)
        self.addCleanup(time.sleep, 0.3)
        self.addCleanup(u.close)
        u.target = ("127.0.0.1", 15104)
        t0 = time.perf_counter()
        for _ in range(50):
            u.send(b"x" * 400)
        dt = time.perf_counter() - t0
        self.assertGreaterEqual(dt, (20000 - 480 - udp_in.CMD_BURST) / udp_in.CMD_RATE)
        self.assertLess(dt, 2.0)
        self.assertEqual(sum(len(board.recv(2048)) for _ in range(50)), 20000)
        time.sleep(0.3)
        t0 = time.perf_counter()
        u.send(b"HELLO\n")                                  # tek komut beklemez
        self.assertLess(time.perf_counter() - t0, 0.02)

    def test_board_not_listening(self):
        """Karttaki telem_udp kapalıyken yazmak alma thread'ini öldürmez (Windows 10054)."""
        u = udp_in.UdpLines(PORT + 5, board="127.0.0.1")
        got = []
        u.on_line = got.append
        self.addCleanup(time.sleep, 0.3)
        self.addCleanup(u.close)
        u.target = ("127.0.0.1", 15199)
        u.send(b"HELLO\n")
        time.sleep(0.3)
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(tx.close)
        tx.sendto(lines_of(b"A\n", 0)[0], ("127.0.0.1", PORT + 5))
        time.sleep(0.3)
        self.assertEqual(got, [b"A\n"])


class Rate(unittest.TestCase):
    def test_h6_instant_rate(self):
        """100 Hz'in 60 s'sinden sonra 20 Hz: anlık hız 2 s içinde 20 ± 1."""
        st, now = TelemetryStats(), 0.0
        for i in range(6000 + 40):
            now += 0.01 if i < 6000 else 0.05
            t = direct(i)
            t.rx_time = now
            st.add(t)
            if i == 5999:
                self.assertAlmostEqual(st.rate_now(now), 100.0, delta=1)
        self.assertAlmostEqual(st.rate_now(now), 20.0, delta=1)
        self.assertGreater(st.rate_hz(), 90)                 # eski ortalama hâlâ 100'e yakın: sorun buydu
        self.assertEqual(st.rate_now(now + 5), 0.0)          # akış durdu


class FakeSsh:
    def __init__(self):
        self.returncode = None

    class stdout:
        @staticmethod
        def read():
            return "telem_udp: aygit yok"

    def poll(self):
        return self.returncode

    def terminate(self):
        self.returncode = 0

    def wait(self):
        pass


class Session(unittest.TestCase):
    def test_restart_until_stop(self):
        """telem_udp biterse RETRY_S sonra yeniden başlar; Durdur'dan sonra başlamaz."""
        made = []
        s = src.TelemSource(PORT)
        self.addCleanup(time.sleep, 0.3)
        self.addCleanup(s.close)
        ses = src.BoardSession(s, run=lambda pc: made.append(FakeSsh()) or made[-1])
        self.assertEqual(ses.start(), "")
        made[0].returncode = 2
        ses.poll(100.0)
        self.assertIn("çıkış 2", ses.text)
        ses.poll(100.0 + src.RETRY_S - 0.1)
        self.assertEqual(len(made), 1)
        ses.poll(100.0 + src.RETRY_S)
        self.assertEqual(len(made), 2)
        ses.stop()
        ses.poll(1000.0)
        self.assertEqual((len(made), made[1].returncode), (2, 0))


class Ports(unittest.TestCase):
    """G1: F746'nın ST-LINK'i seçilir, MP257'ninki seçilmez."""

    def test_guess(self):
        from types import SimpleNamespace as P
        from unittest import mock
        from vnet_host import link
        v3a, v3b = P(device="COM9", vid=0x0483, pid=0x3753), P(device="COM11", vid=0x0483, pid=0x3753)
        f746, bt = P(device="COM6", vid=0x0483, pid=0x3752), P(device="COM4", vid=None, pid=None)
        with mock.patch.object(link.serial.tools.list_ports, "comports") as c:
            for ports, want in (([v3a, bt, v3b, f746], "COM6"), ([v3a, v3b, bt], None), ([], None),
                                ([P(device="COM7", vid=0x0483, pid=0x374B), v3a], "COM7")):
                c.return_value = ports
                self.assertEqual(link.guess_stlink_port(), want)


class FakeLinux:
    """telem_udp + M33 + F746 taklidi: komutlar simülatöre, satırlar PORT + 1'e, TELEM PORT'a."""

    def __init__(self):
        import threading
        from vnet_host.sim_board import SimBoard
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 15103))
        self.sock.settimeout(0.1)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.board = SimBoard(seed=3)
        self.board.attach(self._out)
        self.seq = self.lseq = 0
        self.alive = True
        self.threads = [threading.Thread(target=f, daemon=True) for f in (self._commands, self._telem)]
        for t in self.threads:
            t.start()
        self.returncode = None

    def _out(self, data):
        if self.alive:
            for d in lines_of(data, self.lseq):
                self.lseq += 1
                self.tx.sendto(d, ("127.0.0.1", PORT + 1))

    def _commands(self):
        while self.alive:
            try:
                self.board.feed(self.sock.recv(2048))
            except OSError:
                pass

    def _telem(self):
        while self.alive:
            self.seq += 1
            self.tx.sendto(packet(self.seq), ("127.0.0.1", PORT))
            time.sleep(0.01)

    def poll(self):
        return None

    def terminate(self):
        self.alive = False
        for t in self.threads:
            t.join()
        self.sock.close()
        self.board.stop()

    def wait(self):
        pass


class Gui(unittest.TestCase):
    """Gerçek App, simülatör ve UDP: sekmeler aynı şeyi gösterir, seri taraf onları durdurmaz."""

    def setUp(self):
        from vnet_host import gui
        udp_in_port, src.udp_in.PORT = udp_in.PORT, PORT
        self.addCleanup(setattr, udp_in, "PORT", udp_in_port)
        self.old = src.TelemSource.__init__.__defaults__
        src.TelemSource.__init__.__defaults__ = (PORT,)
        self.addCleanup(setattr, src.TelemSource.__init__, "__defaults__", self.old)
        self.root = tk.Tk()
        self.app = gui.App(self.root)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0

    def tearDown(self):
        self.tx.close()
        self.app._disconnect()
        self.app._close()
        time.sleep(0.3)

    def run_for(self, seconds, udp=True, roll=1234):
        end = time.perf_counter() + seconds
        while time.perf_counter() < end:
            if udp:
                self.seq += 1
                self.tx.sendto(packet(self.seq, roll=roll, yaw=9000), ("127.0.0.1", PORT))
            self.root.update()
            time.sleep(0.01)

    def page(self, p):
        self.app.nb.select(p)
        self.run_for(0.5)
        return [p.ang_vars[k].get() for k in ("roll", "pitch", "yaw")]

    def test_h1_h2_h3_h4(self):
        from vnet_host.imu_page import LatencyTable
        self.app._connect_sim()
        self.run_for(2.0)
        self.assertTrue(self.app.h.start().ok)               # simülatör seri hattan TELEM yollar
        self.run_for(1.5)
        self.assertGreater(self.app.h.telem.received, 20)     # H1: geliyor ama gösterilmiyor
        self.assertEqual(self.app.source.direct_stats.received, 0)
        self.assertGreater(self.app.source.ignored, 20)
        both = [self.page(self.app.flight), self.page(self.app.udp)]
        self.assertEqual(both[0], both[1])                    # H3
        self.assertEqual(both[0], ["+12.34°", "+0.00°", "+90.00°"])
        self.app.udp._zero_yaw()
        self.assertEqual(self.page(self.app.flight)[2], "+0.00°")
        self.assertEqual(self.page(self.app.udp)[2], "+0.00°")
        # H4: tek ortak tablo
        for p in (self.app.flight, self.app.udp):
            self.assertEqual(len([w for w in p.winfo_children() if isinstance(w, LatencyTable)]), 1)
            self.app.nb.select(p)
            self.run_for(0.6)
            self.assertEqual([str(x) for x in p.table.tree.item("whole", "values")][:4], ["619", "619.0", "619", "0"])
        # H2: STOP ve bağlantı kesme Uçuş'u durdurmaz
        self.app.nb.select(self.app.flight)
        self.app.h.stop()
        self.run_for(0.6, roll=2000)
        self.assertEqual(self.app.flight.ang_vars["roll"].get(), "+20.00°")
        self.app._disconnect()
        self.run_for(0.6, roll=3000)
        self.assertEqual(self.app.flight.ang_vars["roll"].get(), "+30.00°")

    def auto(self, linux=True, port=None):
        """Otomatik bağlantı; F746 portu 'port', Linux taklidi ya da hiç başlamayan ssh."""
        from unittest import mock
        self.app.source.lines.target = ("127.0.0.1", 15103)
        self.app.session.run = (lambda pc: FakeLinux()) if linux else (lambda pc: FakeSsh())
        p = mock.patch("vnet_host.gui.linkmod.guess_stlink_port", return_value=port)
        p.start()
        self.addCleanup(p.stop)
        self.assertEqual(self.app.port_var.get(), __import__("vnet_host.gui", fromlist=["x"]).AUTO_PORT)
        self.app._connect_serial()

    def wait_idle(self, limit):
        end = time.perf_counter() + limit
        while self.app.busy and time.perf_counter() < end:
            self.run_for(0.1, udp=False)
        self.run_for(0.3, udp=False)                             # sonuç olay döngüsüyle (60 ms) gelir

    def in_thread(self, fn):
        import threading
        out = []
        t = threading.Thread(target=lambda: out.append(fn()))
        t.start()
        while t.is_alive():
            self.run_for(0.05, udp=False)
        return out[0]

    def test_g2_g4_auto_without_serial(self):
        from vnet_host import link
        self.auto()
        self.wait_idle(10)
        h = self.app.h
        self.assertIsInstance(h.link, link.LinuxLink)
        self.assertTrue(self.app.status_var.get().startswith("Bağlandı"), self.app.status_var.get())
        self.assertTrue(self.in_thread(lambda: h.set_rate(20)).ok)
        self.assertIn("cmd_new", self.in_thread(h.get_counters))
        self.assertEqual((h.link.dropped, h.link.tx_bytes), (0, h.link.linux_tx))
        self.run_for(1.1, udp=False)
        self.assertEqual(self.app.chips["link"].cget("text"), "BAĞLI: MP257 (Linux)")
        self.app.session.stop()                                  # Linux gitti
        self.run_for(1.1, udp=False)
        self.assertEqual(self.app.chips["link"].cget("text"), "LİNUX VERİSİ YOK")

    def test_g3_no_linux_no_serial(self):
        self.auto(linux=False)
        t0 = time.perf_counter()
        worst = 0.0
        while self.app.busy and time.perf_counter() - t0 < 15:
            t = time.perf_counter()
            self.root.update()
            worst = max(worst, time.perf_counter() - t)
            time.sleep(0.01)
        self.run_for(0.3, udp=False)
        self.assertIn("Linux'tan veri gelmiyor", self.app.status_var.get())
        self.assertLess(worst, 0.5)                              # arayüz donmadı

    def test_g3_g4_no_linux_with_serial(self):
        from unittest import mock
        from vnet_host import link
        from vnet_host.sim_board import SimBoard

        class FakeSerial(link.SimLink):
            def __init__(self, port, baud):
                super().__init__(SimBoard(seed=3))
                self.name = "%s @ %d" % (port, baud)

        with mock.patch("vnet_host.gui.linkmod.SerialLink", FakeSerial):
            self.auto(linux=False, port="COM6")
            self.wait_idle(20)
        self.assertTrue(self.app.status_var.get().startswith("Bağlandı"), self.app.status_var.get())
        self.run_for(1.1, udp=False)
        self.assertEqual(self.app.chips["link"].cget("text"), "BAĞLI: COM6 @ 1000000")
        self.assertEqual(self.app.h.link.linux_tx, 0)

    def test_direct_without_linux(self):
        """Linux yokken (UDP gelmiyor) doğrudan TELEM gösterilir."""
        self.app.nb.select(self.app.flight)
        self.app._connect_sim()
        self.run_for(1.5, udp=False)
        self.assertTrue(self.app.h.start().ok)
        self.run_for(1.0, udp=False)
        self.assertEqual(self.app.source.active(), src.DIRECT)
        self.assertNotEqual(self.app.flight.ang_vars["roll"].get(), "—")
        self.assertIn("doğrudan", self.app.flight.src_label.cget("text"))
        self.assertNotEqual(self.app.flight.table.tree.item("direct", "values")[0], "—")


if __name__ == "__main__":
    unittest.main()
