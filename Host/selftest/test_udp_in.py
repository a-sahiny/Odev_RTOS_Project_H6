"""M8 UDP girişi (udp_in.UdpTelem): M33 biçiminde paketler localhost'tan yollanır.

Çalıştırma (Host klasöründen):  python -m unittest discover -s selftest -v
"""

import os
import socket
import sys
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from vnet_host import proto, udp_in  # noqa: E402

PORT = 15002                    # testte kartın portu yerine boş bir port


def telem(seq, roll_cdeg=0):
    """M33'ün packet.c'sinin kurduğu 33 bayt: TELEM, bayrak 0, REQ_ID 0, 23 baytlık ham mesaj."""
    raw = proto.payload_telem((0, 0, 8192), (0, 0, 0), (roll_cdeg, -250, 1000), 0x07, (seq - 1) & 0xFFFF, 596)
    return proto.encode(proto.Packet(proto.TELEM, 0, seq, 0, raw))


class UdpIn(unittest.TestCase):
    def setUp(self):
        self.got = []
        self.rx = udp_in.UdpTelem(PORT, on_telem=self.got.append)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.tx.close()
        self.rx.close()
        time.sleep(0.3)         # okuma thread'i soketi kapatsın

    def send(self, packets):
        for p in packets:
            self.tx.sendto(p, ("127.0.0.1", PORT))
        time.sleep(0.3)

    def test_u1_in_order(self):
        self.send(telem(s, s) for s in range(200))
        t = self.rx.telem
        self.assertEqual((t.received, t.gaps, self.rx.bad), (200, 0, 0))
        self.assertEqual((t.last.seq, t.last.roll, t.last.pitch, t.last.yaw), (199, 1.99, -2.5, 10.0))
        self.assertEqual(len(self.got), 200)

    def test_u2_gap(self):
        self.send(telem(s) for s in list(range(10)) + list(range(13, 20)))
        self.assertEqual((self.rx.telem.received, self.rx.telem.gaps), (17, 3))

    def test_u3_bad(self):
        bad_crc = bytearray(telem(5))
        bad_crc[-1] ^= 0x01
        self.send([telem(4), bytes(bad_crc), telem(5)[:-11] + telem(5)[-10:], telem(5)])
        self.assertEqual((self.rx.telem.received, self.rx.bad, self.rx.telem.gaps), (2, 2, 0))

    def test_u4_port_busy(self):
        with self.assertRaises(OSError):
            udp_in.UdpTelem(PORT)


if __name__ == "__main__":
    unittest.main()
