"""T05 Tekrarlanan komut (tasarım 16.3, 16.4, 20.2).

Aynı REQ_ID + aynı START iki kez: ikincisi yan etkisiz, aynı STATUS (cmd_replay).
Aynı REQ_ID + farklı SET_RATE: E_CONFLICT, hız ilk değerde (cmd_conflict).
Hatalı istek de kaydedilir (D19): aynı hatalı istek aynı hatayı alır.
Kartta test edilen: cmd.c dedup penceresi (8 kayıt, 10 s), prvValidate / prvExecute ayrımı.
"""

import time

from vnet_host import proto
from tests.common import TestResult, prepare, counters
from vnet_host import accounting as acc


def run(h, **_):
    res = TestResult("T05", "Tekrarlanan komut")
    prepare(h, res, rate=10)
    c0 = counters(h)

    # Yeni bir START ölçümü baştan başlatır: ilk 1 s gyro bias'ı ölçülür, TELEM'de BIAS_READY yoktur.
    # Tekrar (aynı REQ_ID) yan etkisizse BIAS_READY ikinci START'tan sonra da set kalır.
    rid = h.next_req_id()
    r1 = h.start(req_id=rid)
    time.sleep(1.4)
    bias_after_first = h.telem.last.bias_ready if h.telem.last is not None else None
    seq_mark = h.telem.last.seq if h.telem.last is not None else None
    r2 = h.start(req_id=rid)
    time.sleep(0.3)
    after = [t for t in list(h.telem_log) if seq_mark is not None and 0 < ((t.seq - seq_mark) & 0xFFFF) < 0x8000]
    bias_kept = bool(after) and all(t.bias_ready for t in after)
    res.check("START REQ_ID %d iki kez" % rid, "ikisi de OK, aynı STATUS; ikinci START ölçümü baştan başlatmaz",
              "%s / %s; BIAS_READY ilk START'tan 1,4 s sonra %s, ikinci START'tan sonraki %d TELEM'de %s" % (
                  r1.status_name, r2.status_name, bias_after_first, len(after),
                  "hep set" if bias_kept else "düştü (ölçüm yeniden başladı)"),
              r1.ok and r2.ok and bias_after_first is True and bias_kept)

    rid2 = h.next_req_id()
    a = h.set_rate(20, req_id=rid2)
    b = h.set_rate(20, req_id=rid2)
    c = h.set_rate(50, req_id=rid2)
    probe = h.set_rate(0)                       # aralık dışı: hız değişmez, o anki hızı söyler
    res.check("SET_RATE 20 / 20 / 50 aynı REQ_ID %d" % rid2,
              "OK, OK (tekrar), E_CONFLICT; hız 20'de kalır",
              "%s, %s, %s; hız %s" % (a.status_name, b.status_name, c.status_name, probe.applied_rate),
              a.ok and b.ok and c.status == proto.ST_E_CONFLICT and probe.applied_rate == 20)

    rid3 = h.next_req_id()
    e1 = h.set_rate(77, req_id=rid3)
    e2 = h.set_rate(77, req_id=rid3)
    e3 = h.set_rate(50, req_id=rid3)
    res.check("hatalı SET_RATE 77 iki kez, sonra aynı REQ_ID ile düzeltilmiş 50 (D19)",
              "E_RANGE, E_RANGE (tekrar), E_CONFLICT", "%s, %s, %s" % (e1.status_name, e2.status_name, e3.status_name),
              e1.status == e2.status == proto.ST_E_RANGE and e3.status == proto.ST_E_CONFLICT)

    h.stop()
    c1 = counters(h)
    d = acc.delta(c0, c1)
    # START, SET_RATE 20, SET_RATE 0 (probe), SET_RATE 77, STOP, + GET_STATS sayfaları (c1'in 3'ü, c0'ın 2'si)
    res.check("dedup sayaçları", "Δcmd_replay = 3, Δcmd_conflict = 2",
              "Δcmd_replay=%d, Δcmd_conflict=%d, Δcmd_new=%d, Δcmd_invalid=%d" % (
                  d["cmd_replay"], d["cmd_conflict"], d["cmd_new"], d["cmd_invalid"]),
              d["cmd_replay"] == 3 and d["cmd_conflict"] == 2)
    h.set_rate(proto.RATE_DEFAULT_HZ)
    return res
