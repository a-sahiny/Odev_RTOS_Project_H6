"""Host kodlayıcı / çözücü birim testleri: tasarım 15.4 örnek baytları ve kartın kuralları.

Çalıştırma (Host klasöründen):  python -m unittest discover -s selftest -v
"""

import os
import random
import struct
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from vnet_host import crc, ethframe as eth, lines, proto, stats_ids  # noqa: E402

BOARD_MAC = bytes.fromhex("0280E1123456")


def hx(s):
    return bytes.fromhex(s.replace(" ", ""))


class Crc(unittest.TestCase):
    def test_check_values(self):
        self.assertEqual(crc.crc8(b"123456789"), 0xF4)
        self.assertEqual(crc.crc32_eth(b"123456789"), 0xCBF43926)

    def test_fcs_residue(self):
        data = bytes(range(60))
        self.assertTrue(crc.fcs_ok(data + crc.fcs_bytes(data)))
        self.assertFalse(crc.fcs_ok(data + b"\x00\x00\x00\x00"))


class DesignExamples(unittest.TestCase):
    """Tasarım 15.4 tablosundaki baytlar (SEQ / REQ_ID tablodaki gibi)."""

    def enc(self, t, flags, seq, req, payload=b""):
        return proto.encode(proto.Packet(t, flags, seq, req, payload))

    def test_requests(self):
        self.assertEqual(self.enc(proto.CMD_START, 0, 1, 0x1001), hx("A5 01 01 00 00 01 10 01 00 CE"))
        self.assertEqual(self.enc(proto.CMD_STOP, 0, 2, 0x1002), hx("A5 01 02 00 00 02 10 02 00 AD"))
        self.assertEqual(self.enc(proto.CMD_SET_RATE, 0, 3, 0x1003, proto.payload_set_rate(20)),
                         hx("A5 01 03 00 00 03 10 03 02 00 14 0D"))
        self.assertEqual(self.enc(proto.CMD_SET_RATE, 0, 4, 0x1004, proto.payload_set_rate(0)),
                         hx("A5 01 03 00 00 04 10 04 02 00 00 DC"))
        self.assertEqual(self.enc(proto.CMD_GET_STATS, 0, 6, 0x1006, proto.payload_get_stats(0)),
                         hx("A5 01 04 00 00 06 10 06 01 00 11"))

    def test_responses(self):
        ok = self.enc(proto.CMD_START, 1, 1, 0x1001, proto.payload_response(0, 0x4D2A, 3, 850))
        self.assertEqual(ok, hx("A5 01 01 01 00 01 10 01 06 00 4D 2A 03 03 52 E7"))
        st = self.enc(proto.CMD_START, 3, 1, 0x1001, proto.payload_response(3, 0x4D2A, 5, 640))
        self.assertEqual(st, hx("A5 01 01 03 00 01 10 01 06 03 4D 2A 05 02 80 7E"))
        r = proto.parse_response(proto.decode(hx("A5 01 03 01 00 02 10 03 08 00 4D 2A 03 03 8E 00 14 1B")))
        self.assertEqual((r.status, r.epoch, r.state, r.lat_us, r.applied_rate), (0, 0x4D2A, 3, 910, 20))
        r = proto.parse_response(proto.decode(hx("A5 01 03 03 00 03 10 04 08 02 4D 2A 03 02 BC 00 14 93")))
        self.assertEqual((r.status_name, r.applied_rate), ("E_RANGE", 20))

    def test_low_and_telem(self):
        # Tablodaki ENV / BULK satırlarında SEQ baytları "01 00" / "02 00" yazılmış (= 0x0100 / 0x0200).
        self.assertEqual(self.enc(proto.ENV, 0, 0x0100, 0, proto.payload_env([100, -100])),
                         hx("A5 01 20 00 01 00 00 00 05 02 00 64 FF 9C 15"))
        self.assertEqual(self.enc(proto.BULK, 0, 0x0200, 0, proto.payload_bulk(0x12345678, 8)),
                         hx("A5 01 21 00 02 00 00 00 0C 12 34 56 78 78 79 7A 7B 7C 7D 7E 7F 8F"))
        # CAL satırı tabloda "00 03 ... FA" yazılmış ama FA, SEQ baytları "03 00" iken doğru;
        # "00 03" ile CRC8 EE olur. Tablo satırı kendi içinde tutarsız: CRC'si tutan biçim test edilir.
        self.assertEqual(self.enc(proto.CAL, 0, 0x0300, 0, proto.payload_cal([10, -5, 0, 3, -2, 1])),
                         hx("A5 01 22 00 03 00 00 00 0C 00 0A FF FB 00 00 00 03 FF FE 00 01 FA"))
        self.assertEqual(self.enc(proto.CAL, 0, 3, 0, proto.payload_cal([10, -5, 0, 3, -2, 1]))[-1], 0xEE)
        tel = hx("A5 01 30 00 00 00 00 00 14 00 00 00 01 00 01 E2 40 00 0A FF EC 40 00 00 01 00 02 00 03 B2")
        t = proto.parse_telem(proto.decode(tel))
        self.assertEqual((t.seq, t.index, t.tick, t.accel, t.gyro), (0, 1, 123456, (10, -20, 16384), (1, 2, 3)))
        self.assertEqual(self.enc(proto.TELEM, 0, 0, 0, proto.payload_telem(1, 123456, (10, -20, 16384), (1, 2, 3))),
                         tel)

    def test_uart_lines(self):
        pkt = self.enc(proto.CMD_START, 0, 1, 0x1001)
        fr = eth.build_udp_frame(BOARD_MAC, eth.HOST_MAC, eth.HOST_IP, eth.BOARD_IP, 6000, 5000, pkt)
        base = ("0280E11234560200000000010800450000260001000040119572C0A83201C0A8320217701388001238AC"
                "A50101000001100100CE0000000000000000")
        self.assertEqual(lines.frame_line(fr, None), ("FRAME " + base + "\n").encode())
        self.assertEqual(lines.frame_line(fr, 0), ("FRAME 0 " + base + "\n").encode())
        self.assertEqual(lines.frame_line(fr, 1), ("FRAME 1 " + base + "90151130\n").encode())

    def test_board_reply_line(self):
        text = ("FRAME 1 0200000000010280E112345608004500002C000100004011956CC0A83202C0A8320113881770"
                "00189058A50101010001100106004D2A030352E700009B400E8C")
        bl = lines.parse_board_line(text)
        self.assertEqual(bl.kind, "frame")
        f = eth.parse_frame(bl.frame, eth.HOST_MAC)
        self.assertTrue(f.ok, f.errors)
        self.assertEqual((f.ip_src, f.sport, f.dport), (eth.BOARD_IP, 5000, 6000))
        r = proto.parse_response(proto.decode(f.udp_payload))
        self.assertEqual((r.req_id, r.status, r.lat_us), (0x1001, 0, 850))


class Decode(unittest.TestCase):
    def test_reject_reasons_in_board_order(self):
        good = proto.encode(proto.Packet(proto.CMD_SET_RATE, 0, 1, 7, b"\x00\x14"))
        cases = {
            "short": good[:9],
            "sof": b"\x5A" + good[1:],
            "version": good[:1] + b"\x02" + good[2:],
            "length": good + b"\x00",
            "crc": good[:-1] + bytes([good[-1] ^ 1]),
            "flags": proto.encode_raw(proto.CMD_SET_RATE, 0x04, 1, 7, b"\x00\x14"),
        }
        for reason, data in cases.items():
            with self.assertRaises(proto.PacketError) as cm:
                proto.decode(data)
            self.assertEqual(cm.exception.reason, reason)

    def test_single_bit_flips_never_accepted(self):
        good = proto.encode(proto.Packet(proto.CMD_SET_RATE, 0, 1, 7, b"\x00\x14"))
        for i in range(len(good) * 8):
            bad = bytearray(good)
            bad[i // 8] ^= 1 << (i % 8)
            with self.assertRaises(proto.PacketError):
                proto.decode(bytes(bad))

    def test_round_trip_random(self):
        rng = random.Random(5)
        for _ in range(3000):
            p = proto.Packet(rng.randrange(256), rng.choice((0, 1, 2, 3)), rng.randrange(65536),
                             rng.randrange(65536), bytes(rng.randrange(256) for _ in range(rng.randrange(201))))
            q = proto.decode(proto.encode(p))
            self.assertEqual((q.type, q.flags, q.seq, q.req_id, q.payload), (p.type, p.flags, p.seq, p.req_id, p.payload))


class Frames(unittest.TestCase):
    def test_checksums_detected(self):
        args = (BOARD_MAC, eth.HOST_MAC, eth.HOST_IP, eth.BOARD_IP, 6000, 5000, b"\xA5" * 12)
        self.assertEqual(eth.parse_frame(eth.build_udp_frame(*args), BOARD_MAC).errors, [])
        self.assertIn("ip_csum", eth.parse_frame(eth.build_udp_frame(*args, bad_ip_checksum=True), BOARD_MAC).errors)
        self.assertIn("udp_csum", eth.parse_frame(eth.build_udp_frame(*args, bad_udp_checksum=True), BOARD_MAC).errors)
        self.assertIn("frame_dst_mac", eth.parse_frame(eth.build_udp_frame(*args), bytes(6)).errors)

    def test_arp_and_icmp(self):
        a = eth.parse_frame(eth.build_arp(1, eth.HOST_MAC, eth.HOST_IP, bytes(6), eth.BOARD_IP), BOARD_MAC)
        self.assertEqual((a.kind, a.arp_op, a.arp_target_ip), ("ARP", 1, eth.BOARD_IP))
        i = eth.parse_frame(eth.build_icmp_echo(BOARD_MAC, eth.HOST_MAC, eth.HOST_IP, eth.BOARD_IP, 1, 2), BOARD_MAC)
        self.assertEqual((i.kind, i.icmp_type, i.icmp_seq, i.errors), ("ICMP", 8, 2, []))

    def test_minimum_length(self):
        fr = eth.build_arp(1, eth.HOST_MAC, eth.HOST_IP, bytes(6), eth.BOARD_IP)
        self.assertEqual(len(fr), 60)


class CounterTable(unittest.TestCase):
    def test_matches_firmware_header(self):
        hdr = os.path.join(os.path.dirname(os.path.dirname(HERE)), "App", "Inc", "app_stats.h")
        if not os.path.exists(hdr):
            self.skipTest("app_stats.h yok")
        self.assertEqual(stats_ids.parse_header(hdr), stats_ids.COUNTER_NAMES)
        self.assertEqual(len(stats_ids.COUNTER_NAMES), 128)

    def test_imu_reply_layout(self):
        """GET_IMU cevabı (raporlar 086, 091): 35 B, cmd.c prvWriteImu sırası; prev, t_rx son 8 B."""
        b = proto.payload_imu_reply(77, 0x12345, (8000, 8001, 8002), (-8000, -8001, -8002), (4500, -1234, -18000), 3,
                                    0xDEADBEEF, 0x0200, 4700)
        self.assertEqual(len(b), proto.IMU_REPLY_LEN)
        self.assertEqual(b[-8:], bytes.fromhex("0200125CDEADBEEF"))
        self.assertEqual(b[20:27], bytes.fromhex("1194FB2EB9B003"))
        r = proto.parse_imu_reply(b)
        self.assertEqual((r.index, r.tick, r.accel, r.gyro, r.roll, r.pitch, r.yaw, r.flags, r.t_rx),
                         (77, 0x12345, (8000, 8001, 8002), (-8000, -8001, -8002), 45.0, -12.34, -180.0, 3, 0xDEADBEEF))
        self.assertEqual((r.prev_req_id, r.prev_us), (0x0200, 4700))
        self.assertTrue(r.bias_ready and r.acc_used)
        with self.assertRaises(proto.PacketError):
            proto.parse_imu_reply(b[:-1])

    def test_stats_page_parse(self):
        extra = bytes([1]) + struct.pack(">I", 777) + bytes([3]) + struct.pack(">3I", 5, 6, 7)
        sp = proto.parse_stats_page(extra)
        self.assertEqual((sp.page, sp.tick, sp.values), (1, 777, [5, 6, 7]))


if __name__ == "__main__":
    unittest.main()
