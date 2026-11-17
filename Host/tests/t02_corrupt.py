"""T02 Bozuk giriş (tasarım 14, 16.2, 20.2).

Her bozuk girdiden sonra geçerli bir komut gönderilir. Kontrol: bozuk girdi yan etki
üretmez (hız değişmez), kendi ret nedeninde sayılır (rx_err / error raporu + sayaç),
ardından gelen geçerli komut işlenir.
Kartta test edilen: line_rx (hex, uzunluk, timeout, bayrak), vnetRx frame kontrolleri
(FCS, IP / UDP checksum), pkt_parser (CRC8, tür <-> port), cmd.c (E_LEN, E_TYPE, REQ_ID 0,
aralık), console.c (bilinmeyen komut, HOLD sınırı).
"""

import time

from vnet_host import ethframe as eth, lines, proto
from tests.common import TestResult, prepare, counters


def _cmd_frame(h, hz, req_id=None, **kw):
    req_id, pkt = h.command_packet(proto.CMD_SET_RATE, proto.payload_set_rate(hz), req_id=req_id)
    return req_id, h.udp_frame(proto.PORT_CTRL, pkt, **kw)


def run(h, **_):
    res = TestResult("T02", "Bozuk giriş")
    prepare(h, res, rate=10)
    good_hz = [14, 15]
    current = {"hz": 10}

    def case(label, send, counter, report_reason=None, report_type="rx_err", expect_reply=None):
        c0 = counters(h)
        mark = h.report_mark()
        reply = send()
        rep = None
        if report_reason:
            rep = h.wait_report(report_type, mark, 1.5, pred=lambda r: r.get("reason") == report_reason)
        c1 = counters(h)
        dc = (c1[counter] - c0[counter]) & 0xFFFFFFFF
        # Yan etki yok mu? Aralık dışı SET_RATE hiçbir şey değiştirmez ve o anki hızı döndürür.
        probe = h.set_rate(0).applied_rate
        hz = good_hz[0]
        good_hz.reverse()
        follow = h.set_rate(hz)
        ok_reply = True if expect_reply is None else expect_reply(reply)
        ok = (dc == 1 and (rep is not None or not report_reason) and probe == current["hz"]
              and follow.ok and follow.applied_rate == hz and ok_reply)
        res.check(label,
                  "Δ%s=1%s; hız %d olarak kalır; ardından SET_RATE %d OK" % (
                      counter, ", rapor %s:%s" % (report_type, report_reason) if report_reason else "",
                      current["hz"], hz),
                  "Δ%s=%d, rapor %s, hız %s, sonraki: %s hız %s%s" % (
                      counter, dc, rep.get("reason") if rep else "yok", probe, follow.status_name,
                      follow.applied_rate, "" if reply is None else ", cevap: %s" % reply), ok)
        current["hz"] = hz

    # 1) hex olmayan karakter
    case("FRAME 1 + hex olmayan karakter",
         lambda: h.send_line(b"FRAME 1 0280E1ZZ" + b"00" * 40 + b"\n"), "line_bad_hex", "line_bad_hex")
    # 2) tek sayıda hex
    case("FRAME 1 + tek sayıda hex karakteri",
         lambda: h.send_line(b"FRAME 1 " + b"A" * 129 + b"\n"), "line_bad_hex", "line_bad_hex")
    # 3) 2000 karakterlik satır
    case("2000 karakterlik satır", lambda: h.send_line(b"FRAME 1 " + b"AB" * 996 + b"\n"),
         "line_too_long", "line_too_long")

    # 4) '\n'i 1 s gecikmiş satır: ilk parça timeout'a uğrar, geç gelen kuyruk anlamsız satır olur
    def delayed():
        _, fr = _cmd_frame(h, 40)
        line = lines.frame_line(fr, 1)
        # 1 s'lik sessizlik boyunca hattı ayır (rapor 092): araya başka satır girerse sessizlik olmaz.
        with h.link.hold():
            h.send_line(line[:60])
            time.sleep(1.0)
            h.send_line(line[60:])
    case("SET_RATE 40 satırının '\\n'i 1 s gecikiyor", delayed, "line_timeout", "line_timeout")

    # 5) bayrak 2
    def flag2():
        _, fr = _cmd_frame(h, 41)
        h.send_line(b"FRAME 2 " + fr.hex().upper().encode() + b"\n")
    case("FRAME 2 <hex> (geçersiz bayrak)", flag2, "line_bad_flag", "line_bad_flag")

    # 6) FRAME 1 + bozuk FCS
    def bad_fcs():
        _, fr = _cmd_frame(h, 42)
        line = bytearray(lines.frame_line(fr, 1))
        line[-3] = ord("0") if line[-3] != ord("0") else ord("1")
        h.send_line(bytes(line))
    case("FRAME 1 + bozuk FCS", bad_fcs, "frame_fcs", "frame_fcs")

    # 7) bozuk IPv4 başlık checksum'ı
    case("bozuk IPv4 başlık checksum'ı",
         lambda: h.send_frame(_cmd_frame(h, 43, bad_ip=True)[1]), "ip_csum", "ip_csum")
    # 8) bozuk UDP checksum'ı
    case("bozuk UDP checksum'ı",
         lambda: h.send_frame(_cmd_frame(h, 44, bad_udp=True)[1]), "udp_csum", "udp_csum")
    # 9) yanlış EtherType
    def ethertype():
        fr = bytearray(_cmd_frame(h, 45)[1])
        fr[12:14] = b"\x86\xDD"
        h.send_frame(bytes(fr))
    case("EtherType 0x86DD", ethertype, "frame_ethertype", "frame_ethertype")
    # 10) başka MAC'e giden frame
    case("hedef MAC başka bir kart",
         lambda: h.send_frame(_cmd_frame(h, 46, dst_mac=bytes.fromhex("02AABBCCDDEE"))[1]),
         "frame_dst_mac", "frame_dst_mac")
    # 11) kısa frame (FCS hariç 40 bayt)
    case("40 baytlık frame", lambda: h.send_line(b"FRAME 0 " + b"00" * 40 + b"\n"), "frame_length", "frame_length")

    # 12) uygulama paketi: bozuk CRC8 -> cevapsız ret
    def bad_crc():
        req_id, pkt = h.command_packet(proto.CMD_SET_RATE, proto.payload_set_rate(47))
        pkt = pkt[:-1] + bytes([pkt[-1] ^ 0xFF])
        h.send_udp(proto.PORT_CTRL, pkt)
    case("SET_RATE 47, CRC8 bozuk", bad_crc, "pkt_crc_crit")

    # 13) SET_RATE LEN=1 (tasarım 15.4): eksik komut -> E_LEN cevabı, hiçbir şey başlamaz
    def short_len():
        return h.command(proto.CMD_SET_RATE, b"\x14").status_name
    case("SET_RATE LEN=1 (eksik payload)", short_len, "cmd_invalid", expect_reply=lambda r: r == "E_LEN")

    # 13b) hiçbir lane'in bilmediği TYPE komut portunda -> E_TYPE cevabı
    def unknown_type():
        return h.command(0x55, b"").status_name
    case("bilinmeyen TYPE 0x55, port 5000", unknown_type, "cmd_invalid", expect_reply=lambda r: r == "E_TYPE")

    # 14) ENV komut portuna (tür <-> port)
    def wrong_port():
        raw = proto.encode(proto.Packet(proto.ENV, 0, h.next_seq("env"), 0, proto.payload_env([1])))
        h.send_udp(proto.PORT_CTRL, raw)
    case("ENV paketi 5000'e (yanlış port)", wrong_port, "pkt_wrong_port_crit")

    # 15) REQ_ID 0 -> E_REQID cevabı
    def reqid0():
        return h.command(proto.CMD_SET_RATE, proto.payload_set_rate(48), req_id=0).status_name
    case("SET_RATE 48, REQ_ID 0", reqid0, "cmd_invalid", expect_reply=lambda r: r == "E_REQID")

    # 16) aralık dışı hız -> E_RANGE, hız değişmez
    def out_of_range():
        return h.set_rate(0).status_name
    case("SET_RATE 0 Hz", out_of_range, "cmd_invalid", expect_reply=lambda r: r == "E_RANGE")

    # 17) konsol: bilinmeyen komut ve HOLD sınırı
    case("konsol 'HELO'", lambda: h.send_line(b"HELO\n"), "line_unknown", "unknown_command", "error")
    case("konsol 'HOLD 99999'", lambda: h.send_line(b"HOLD 99999\n"), "line_bad_arg", "bad_arg", "error")

    res.note("Eksik komut (yanlış LEN) ve bilinmeyen TYPE cevapla reddedilir: E_LEN / E_TYPE (tasarım 16.2, "
             "15.4; rapor 085). Cevapsız retler: bozuk paket (CRC8, SOF...) ve başka lane'in türü (PKT_WRONG_PORT).")
    h.set_rate(proto.RATE_DEFAULT_HZ)
    return res
