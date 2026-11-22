"""PC'de çalışan uyumluluk testi: f7common.proto, F746 projesinin kendi host kodu ve kart simülatörüyle aynı mı?

Kart gerekmez. Odev_RTOS_Project_H6/Host/vnet_host içindeki sim_board (F746'nın protokol modeli) kullanılır;
o projeye hiçbir şey yazılmaz. Çalıştırma (PC'de, proje kökünden):
    python -m unittest discover -s linux/tests -v
"""

import os
import sys
import threading
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from f7common import proto  # noqa: E402

F746_HOST = os.environ.get("F746_HOST_DIR", r"C:\Users\Asus\Documents\GitHub\Odev_RTOS_Project_H6\Host")
HAVE_F746 = os.path.isdir(os.path.join(F746_HOST, "vnet_host"))
if HAVE_F746:
    sys.path.insert(0, F746_HOST)
    from vnet_host import crc as h_crc, ethframe as h_eth, lines as h_lines, proto as h_proto  # noqa: E402
    from vnet_host.sim_board import SimBoard  # noqa: E402

HOST_MAC = bytes.fromhex(proto.HOST_MAC.replace(":", ""))


class ProtoTest(unittest.TestCase):
    def test_crc8_check_value(self):
        self.assertEqual(proto.crc8(b"123456789"), 0xF4)

    def test_frame_line_roundtrip_and_padding(self):
        short = bytes(range(42))                                  # ARP boyu: 60'a doldurulmalı
        line = proto.frame_to_line(short)
        frame = proto.line_to_frame(line.rstrip(b"\n"))
        self.assertEqual(len(frame), 60)
        self.assertEqual(frame[:42], short)
        with self.assertRaises(proto.PacketError):
            proto.frame_to_line(bytes(257))                       # MTU 242 + 14 = 256 sınırı

    def test_bad_fcs_rejected(self):
        line = bytearray(proto.frame_to_line(bytes(60)).rstrip(b"\n"))
        line[-1] = ord("0") if line[-1] != ord("0") else ord("1")
        with self.assertRaises(proto.PacketError):
            proto.line_to_frame(bytes(line))


@unittest.skipUnless(HAVE_F746, "Odev_RTOS_Project_H6/Host bulunamadi")
class F746CompatTest(unittest.TestCase):
    def test_same_as_f746_host(self):
        self.assertEqual(proto.crc8(b"abc"), h_crc.crc8(b"abc"))
        self.assertEqual(proto.fcs(b"x" * 60), h_crc.fcs_bytes(b"x" * 60))
        pkt = proto.encode(proto.CMD_SET_RATE, 7, 1234, b"\x00\x14")
        self.assertEqual(pkt, h_proto.encode(h_proto.Packet(type=0x03, seq=7, req_id=1234, payload=b"\x00\x14")))
        self.assertEqual((proto.RATES_HZ, proto.TELEM_LEN), (h_proto.RATES_HZ, h_proto.TELEM_LEN))
        tel = h_proto.payload_telem((10, -20, 16384), (1, 2, 3), (4500, -1234, -18000), 7, 0x0200, 4700)
        t = proto.parse_telem(tel)
        self.assertEqual((t["acc"], t["gyro"], t["roll_cdeg"], t["pitch_cdeg"], t["yaw_cdeg"], t["flags"],
                          t["prev_seq"], t["prev_us"]), ([10, -20, 16384], [1, 2, 3], 4500, -1234, -18000, 7, 0x0200, 4700))
        arp = h_eth.build_arp(1, HOST_MAC, proto.HOST_IP, bytes(6), proto.BOARD_IP)
        self.assertEqual(proto.frame_to_line(arp), h_lines.frame_line(arp, 1))

    def test_end_to_end_with_simulated_f746(self):
        """TAP'ın yerine bu test geçer: çerçeveyi kendisi kurar, f7linkd'nin satır dönüşümünden geçirir."""
        board = SimBoard()
        lines = []
        buf = bytearray()
        lock = threading.Lock()

        def on_out(data):
            with lock:
                buf.extend(data)
                while b"\n" in buf:
                    i = buf.index(b"\n")
                    lines.append(bytes(buf[:i]))
                    del buf[:i + 1]

        board.attach(on_out)
        try:
            def send(frame):
                board.feed(proto.frame_to_line(frame))            # f7linkd: TAP -> F746 yönü

            def udp_payloads(port):
                out = []
                with lock:
                    got = list(lines)
                for ln in got:
                    if not ln.startswith(b"FRAME 1 "):
                        continue
                    f = h_eth.parse_frame(proto.line_to_frame(ln))  # f7linkd: F746 -> TAP yönü
                    if f.ok and f.kind == "UDP" and f.sport == port:
                        out.append(f.udp_payload)
                return out

            # Linux çekirdeğinin yapacağı gibi: önce ARP (kart host MAC'ini öğrensin)
            send(h_eth.build_arp(1, HOST_MAC, proto.HOST_IP, bytes(6), proto.BOARD_IP))
            time.sleep(0.2)
            board_mac = None
            with lock:
                got = list(lines)
            for ln in got:
                if ln.startswith(b"FRAME 1 "):
                    f = h_eth.parse_frame(proto.line_to_frame(ln))
                    if f.ok and f.kind == "ARP" and f.arp_op == 2:
                        board_mac = f.src
            self.assertIsNotNone(board_mac, "ARP cevabi yok")

            def udp(dport, payload):
                return h_eth.build_udp_frame(board_mac, HOST_MAC, proto.HOST_IP, proto.BOARD_IP,
                                             proto.HOST_PORT, dport, payload)

            send(udp(proto.PORT_CTRL, proto.encode(proto.CMD_SET_RATE, 1, 4320, b"\x00\xC8")))   # 200 Hz
            time.sleep(0.2)
            send(udp(proto.PORT_CTRL, proto.encode(proto.CMD_START, 2, 4321)))
            time.sleep(1.5)                                       # START sonrası telemetri aksın
            send(udp(proto.PORT_CTRL, proto.encode(0x05, 3, 4322)))   # eski GET_IMU: artık E_TYPE
            time.sleep(0.3)

            replies = {}
            for p in udp_payloads(proto.PORT_CTRL):
                t, flags, seq, req_id, payload = proto.decode(p)
                if flags & proto.FLAG_RESP:
                    replies[req_id] = (t, proto.parse_response(payload))
            self.assertIn(4320, replies, "SET_RATE cevabi yok")
            self.assertEqual(replies[4320][1]["status"], 0)
            self.assertIn(4321, replies, "START cevabi yok")
            self.assertEqual(replies[4321][1]["status"], 0)
            self.assertTrue(replies[4321][1]["state"] & proto.STATE_MEAS_EN)
            self.assertIn(4322, replies, "eski GET_IMU cevabi yok")
            self.assertEqual(replies[4322][1]["status"], 5)        # E_TYPE

            telem = [proto.decode(p) for p in udp_payloads(proto.PORT_TELEM)]
            self.assertGreater(len(telem), 200, "200 Hz telemetri gelmedi")   # 1,5 s x 200 Hz
            parsed = []
            for t, flags, seq, req_id, payload in telem:
                self.assertEqual(t, proto.TELEM)
                parsed.append(proto.parse_telem(payload))
            self.assertTrue(all(-18000 <= p["roll_cdeg"] <= 18000 for p in parsed))
            lat = [p["prev_us"] for p in parsed[1:]]
            self.assertTrue(all(v is not None and 0 < v <= proto.TELEM_DEADLINE_MS * 1000 for v in lat),
                            "her TELEM bir öncekinin kart içi gecikmesini taşımalı")

            with lock:
                self.assertTrue(any(ln.startswith(b"{") for ln in lines), "JSON raporu yok")
        finally:
            board.stop()


if __name__ == "__main__":
    unittest.main()
