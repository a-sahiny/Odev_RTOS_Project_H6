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
from vnet_host.hoststack import Host, ImuPoller, StatusError  # noqa: E402
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
        self.assertTrue(self.h.set_rate(25).ok)
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
        r3 = self.h.set_rate(30, req_id=500)
        self.assertEqual((r1.status, r2.status, r3.status), (proto.ST_OK, proto.ST_OK, proto.ST_E_CONFLICT))
        self.assertEqual(r3.applied_rate, 20)

    def test_get_imu(self):
        """Rapor 086: GET_IMU yalnız START ile STOP arasında; açılar aralıkta; tekrar penceresine girmez."""
        with self.assertRaises(StatusError) as cm:
            self.h.get_imu(timeout=0.5)
        self.assertEqual(cm.exception.status, proto.ST_E_STATE)
        r = self.h.command(proto.CMD_GET_IMU, payload=b"", timeout=0.5)
        self.assertEqual(r.status, proto.ST_E_LEN)
        self.assertTrue(self.h.start(req_id=900).ok)
        time.sleep(0.05)
        poller = ImuPoller(self.h, period_ms=20.0)
        poller.start()
        time.sleep(1.5)
        poller.stop()
        s = poller.stats
        self.assertGreater(s.ok, 30)
        self.assertEqual(s.timeouts, 0)
        self.assertGreater(s.board_n, 20)              # kartın (simülatörün) M1 -> M6'sı cevaplarla geldi
        self.assertEqual(s.board_n, len(s.board))
        last = s.last
        self.assertTrue(-180 <= last.roll <= 180 and -90 <= last.pitch <= 90 and -180 <= last.yaw <= 180)
        self.assertTrue(last.bias_ready)
        r2 = self.h.start(req_id=900)           # 100+ GET_IMU sonra hâlâ tekrar
        self.assertTrue(r2.ok)
        c = self.h.get_counters()
        self.assertGreaterEqual(c["cmd_replay"], 1)
        self.assertTrue(self.h.stop().ok)
        with self.assertRaises(StatusError):
            self.h.get_imu(timeout=0.5)

    def test_console_and_injection_labels(self):
        self.assertEqual(self.h.hold(300)["result"], "hold_requested")
        self.assertEqual(self.h.txdelay(5)["result"], "txdelay_set")
        st = self.h.state()
        self.assertEqual(st["inj"][0] & 2, 2)
        self.h.txdelay(0)
        self.assertEqual(self.h.console("NOPE", ("error",))["reason"], "unknown_command")


if __name__ == "__main__":
    unittest.main()
