"""Muhasebe (tasarım 18): iki GET_STATS görüntüsü arasındaki farklar ve eşitlikler.

Eşitlikler sakin anda tam tutar (STOP -> HOLD yok -> queue'lar boş -> görüntü).
Akış sürerken farklar en çok queue kapasiteleri kadardır.
"""

from dataclasses import dataclass

from .stats_ids import COUNTER_NAMES


def delta(before: dict, after: dict) -> dict:
    """Sayaç farkları (32-bit sarmaya dayanıklı)."""
    return {n: (after.get(n, 0) - before.get(n, 0)) & 0xFFFFFFFF for n in COUNTER_NAMES}


@dataclass
class Check:
    layer: str
    text: str
    lhs: int
    rhs: int
    exact: bool = True          # False: lhs >= rhs yeter (bekleyen / açıklanan fark)
    tol: int = 0                # |lhs - rhs| <= tol: görüntü anında queue'da bekleyenler

    @property
    def ok(self) -> bool:
        if self.tol:
            return abs(self.lhs - self.rhs) <= self.tol
        return self.lhs == self.rhs if self.exact else self.lhs >= self.rhs


def _s(d, *names):
    return sum(d[n] for n in names)


def equalities(d: dict, host_tx_bytes: int = None, host_rx_lines: int = None):
    """d: delta(). host_*: aynı pencerede host'un saydıkları (L1 / L8 karşılaştırması)."""
    lane = lambda base, sfx: base + "_" + sfx
    out = []
    if host_tx_bytes is not None:
        out.append(Check("L1", "host'un yazdığı bayt = rx_bytes + uart_rx_sb_ovf (+ORE/FE/NE)",
                         host_tx_bytes, d["rx_bytes"], exact=False))
    out.append(Check("L2", "lines_total = frames + console_cmds + Σ line_*",
                     d["lines_total"], _s(d, "frames", "console_cmds", "line_too_long", "line_uart_loss",
                                          "line_timeout", "line_unknown", "line_bad_flag", "line_bad_hex",
                                          "line_bad_arg")))
    out.append(Check("L2", "frames = rx_fcs_none + rx_fcs0 + rx_fcs1",
                     d["frames"], _s(d, "rx_fcs_none", "rx_fcs0", "rx_fcs1")))
    out.append(Check("L2", "frames = to_stack + Σ frame_* + ip_csum + udp_csum + rx_no_netbuf + rx_ipq_full",
                     d["frames"], _s(d, "to_stack", "frame_length", "frame_fcs", "frame_ethertype",
                                     "frame_dst_mac", "ip_csum", "udp_csum", "rx_no_netbuf", "rx_ipq_full")))
    for sfx in ("crit", "low"):
        out.append(Check("L3", "cb_%s = sb_written + sb_full + dgram_bad_len" % sfx,
                         d["cb_" + sfx], _s(d, "sb_written_" + sfx, "cb_%s_sb_full" % sfx, "dgram_bad_len_" + sfx)))
        out.append(Check("L3", "to_stack_%s >= cb_%s (fark = ARP bekletmesi, arp_delayed_full)" % (sfx, sfx),
                         d["to_stack_" + sfx], d["cb_" + sfx], exact=False))
        out.append(Check("L4", "sb_written_%s = rec_read_%s (sakin anda)" % (sfx, sfx),
                         d["sb_written_" + sfx], d["rec_read_" + sfx]))
        errs = [lane(x, sfx) for x in ("pkt_short", "pkt_sof", "pkt_version", "pkt_length", "pkt_crc",
                                       "pkt_flags", "pkt_wrong_port", "pkt_type_len")]
        out.append(Check("L4", "rec_read_%s = pkt_ok + Σ pkt_*" % sfx,
                         d["rec_read_" + sfx], d["pkt_ok_" + sfx] + _s(d, *errs)))
    out.append(Check("L4", "pkt_ok_crit = evt_posted_crit + pool_crit_empty + ctrlq_full",
                     d["pkt_ok_crit"], _s(d, "evt_posted_crit", "pool_crit_empty", "ctrlq_full")))
    out.append(Check("L4", "pkt_ok_low = evt_posted_low + pool_low_empty + workq_full + seq_dup_low + seq_old_low",
                     d["pkt_ok_low"], _s(d, "evt_posted_low", "pool_low_empty", "workq_full", "seq_dup_low",
                                         "seq_old_low")))
    out.append(Check("L5", "evt_posted_crit + evt_net_posted + evt_posted_low = handled_ctrl + handled_work + no_handler",
                     _s(d, "evt_posted_crit", "evt_net_posted", "evt_posted_low"),
                     _s(d, "evt_handled_ctrl", "evt_handled_work", "evt_no_handler")))
    out.append(Check("L5", "evt_handled_ctrl = cmd_new + cmd_replay + cmd_conflict + cmd_invalid + evt_net_posted",
                     d["evt_handled_ctrl"], _s(d, "cmd_new", "cmd_replay", "cmd_conflict", "cmd_invalid",
                                               "evt_net_posted")))
    out.append(Check("L5", "evt_handled_work = env_pkts + env_bad + bulk_pkts + cal_applied + cal_rejected",
                     d["evt_handled_work"], _s(d, "env_pkts", "env_bad", "bulk_pkts", "cal_applied", "cal_rejected")))
    out.append(Check("L6", "telem_gen = telem_sendto_ok + telem_sendto_fail",
                     d["telem_gen"], _s(d, "telem_sendto_ok", "telem_sendto_fail")))
    out.append(Check("L7", "if_out + tx_report_in = txq_ok + txq_full + txq_reserve + tx_bad_len",
                     _s(d, "if_out", "tx_report_in"), _s(d, "txq_ok", "txq_full", "txq_reserve", "tx_bad_len")))
    # GET_STATS cevabının kendisi ve periyodik st / env raporları görüntü anında TX queue'sunda
    # olabilir: fark en çok queue kapasitesi (8) kadardır (tasarım 18.1 kural 3).
    out.append(Check("L7", "txq_ok = tx_lines + tx_uart_stall (fark ≤ TX queue'su 8)",
                     d["txq_ok"], _s(d, "tx_lines", "tx_uart_stall"), tol=8))
    if host_rx_lines is not None:
        out.append(Check("L8", "tx_lines (kart) >= host'un aldığı satır; fark = USB/VCP kaybı",
                         d["tx_lines"], host_rx_lines, exact=False))
    return out


def nonzero(d: dict, names=None):
    names = names or COUNTER_NAMES
    return {n: d[n] for n in names if d.get(n)}
