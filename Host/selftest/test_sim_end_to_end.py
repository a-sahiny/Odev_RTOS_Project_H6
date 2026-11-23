"""Host yığını + kart simülatörü uçtan uca (gerçek kart gerekmez).

Çalıştırma (Host klasöründen):  python -m unittest discover -s selftest -v
"""

import os
import sys
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from vnet_host import accounting as acc, proto  # noqa: E402
from vnet_host.hoststack import Host, TelemLatencyStats  # noqa: E402
from vnet_host.link import SimLink  # noqa: E402
from vnet_host.sim_board import SimBoard  # noqa: E402


class SimEndToEnd(unittest.TestCase):
    def setUp(self):
        self.h = Host(SimLink(SimBoard(seed=3)))
        self.hello = self.h.connect()

    def tearDown(self):
        self.h.link.close()

    def test_hello_arp_ping(self):
        self.assertEqual(self.hello["type"], "hello")
        self.assertIsNotNone(self.h.board_mac)
        self.assertIsNotNone(self.h.ping())

    def test_commands_telemetry_and_accounting(self):
        c0 = self.h.get_counters()
        self.assertTrue(self.h.set_rate(20).ok)
        self.assertTrue(self.h.start().ok)
        time.sleep(0.8)
        self.assertTrue(self.h.stop().ok)
        self.h.send_env([5, -5, 7])
        self.h.send_bulk(99, 40)
        self.h.send_cal([1, 1, 1, 1, 1, 1])
        self.h.drain()
        c1 = self.h.get_counters()
        d = acc.delta(c0, c1)
        self.assertGreater(self.h.telem.received, 10)
        self.assertEqual(self.h.telem.gaps, 0)
        bad = [c for c in acc.equalities(d) if not c.ok]
        self.assertEqual(bad, [], [(c.text, c.lhs, c.rhs) for c in bad])
        self.assertEqual((d["env_pkts"], d["bulk_pkts"], d["cal_applied"]), (1, 1, 1))
        env = self.h.get_env()
        self.assertEqual(env[0]["mean"], 5)

    def test_replay_and_conflict(self):
        r1 = self.h.set_rate(20, req_id=500)
        r2 = self.h.set_rate(20, req_id=500)
        r3 = self.h.set_rate(50, req_id=500)
        self.assertEqual((r1.status, r2.status, r3.status), (proto.ST_OK, proto.ST_OK, proto.ST_E_CONFLICT))
        self.assertEqual(r3.applied_rate, 20)

    def test_set_rate_only_listed(self):
        for hz in proto.RATES_HZ:
            r = self.h.set_rate(hz)
            self.assertEqual((r.status, r.applied_rate), (proto.ST_OK, hz))
        for hz in (0, 1, 25, 30, 150, 201, 1000):
            r = self.h.set_rate(hz)
            self.assertEqual((r.status, r.applied_rate), (proto.ST_E_RANGE, 200))
        self.assertEqual(self.h.command(0x05, b"", timeout=0.5).status, proto.ST_E_TYPE)   # eski GET_IMU

    def test_telem_every_sample_with_latency(self):
        """Her IMU örneği bir TELEM (SET_RATE hızında); her TELEM bir öncekinin kart içi gecikmesini taşır."""
        for hz in (20, 100):
            self.assertTrue(self.h.set_rate(hz).ok)
            self.h.telem.reset()
            lat = TelemLatencyStats()
            got = []
            fn = lambda k, d: k == "telem" and (lat.add(d), got.append(d))
            self.h.listeners.append(fn)
            self.assertTrue(self.h.start().ok)
            time.sleep(1.5)
            self.assertTrue(self.h.stop().ok)
            time.sleep(0.1)
            self.h.listeners.remove(fn)
            self.assertAlmostEqual(self.h.telem.rate_hz(), hz, delta=hz * 0.15)
            self.assertEqual(self.h.telem.gaps, 0)
            self.assertGreater(lat.n, len(got) // 2)
            self.assertEqual(lat.over_deadline, 0)
            self.assertTrue(all(t.prev_valid for t in got[1:]))
            last = got[-1]
            self.assertTrue(-180 <= last.roll <= 180 and -90 <= last.pitch <= 90 and -180 <= last.yaw <= 180)
            self.assertTrue(last.bias_ready)
        c = self.h.get_counters()
        self.assertEqual(c["telem_miss"], 0)

    def test_console_and_injection_labels(self):
        self.assertEqual(self.h.hold(300)["result"], "hold_requested")
        self.assertEqual(self.h.txdelay(5)["result"], "txdelay_set")
        st = self.h.state()
        self.assertEqual(st["inj"][0] & 2, 2)
        self.h.txdelay(0)
        self.assertEqual(self.h.console("NOPE", ("error",))["reason"], "unknown_command")


if __name__ == "__main__":
    unittest.main()
