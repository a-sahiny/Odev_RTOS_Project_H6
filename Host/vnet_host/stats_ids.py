"""GET_STATS sayaç adları: App/Inc/app_stats.h eStatId_t sırası (127 sayaç).

Kart değişirse: python -m vnet_host.stats_ids ../App/Inc/app_stats.h ile yeniden kontrol edilir.
"""

import re

COUNTER_NAMES = (
    "rx_bytes", "uart_rx_sb_ovf", "uart_rx_dma_lag", "uart_rx_tc_after_idle", "uart_ore",
    "uart_fe", "uart_ne", "lines_total", "console_cmds",
    "line_too_long", "line_uart_loss", "line_timeout", "line_unknown",
    "line_bad_flag", "line_bad_hex", "line_bad_arg", "frames",
    "rx_fcs_none", "rx_fcs0", "rx_fcs1", "frame_length",
    "frame_fcs", "frame_ethertype", "frame_dst_mac", "ip_csum",
    "udp_csum", "rx_no_netbuf", "rx_ipq_full", "to_stack",
    "to_stack_crit", "to_stack_low", "cb_crit", "sb_written_crit",
    "cb_crit_sb_full", "dgram_bad_len_crit", "cb_low", "sb_written_low",
    "cb_low_sb_full", "dgram_bad_len_low", "rec_read_crit", "pkt_ok_crit",
    "pkt_short_crit", "pkt_sof_crit", "pkt_version_crit", "pkt_length_crit",
    "pkt_crc_crit", "pkt_flags_crit", "pkt_wrong_port_crit", "pkt_type_len_crit",
    "seq_dup_crit", "seq_gap_crit", "seq_old_crit", "seq_resync_crit",
    "evt_posted_crit", "ctrlq_full", "rec_read_low", "pkt_ok_low",
    "pkt_short_low", "pkt_sof_low", "pkt_version_low", "pkt_length_low",
    "pkt_crc_low", "pkt_flags_low", "pkt_wrong_port_low", "pkt_type_len_low",
    "seq_dup_low", "seq_gap_low", "seq_old_low", "seq_resync_low",
    "evt_posted_low", "workq_full", "pool_crit_empty", "pool_crit_invalid",
    "pool_crit_double", "pool_low_empty", "pool_low_invalid", "pool_low_double",
    "pool_report_empty", "pool_report_invalid", "pool_report_double", "pool_unregistered",
    "evt_handled_ctrl", "evt_handled_work", "evt_no_handler", "evt_net_posted",
    "evt_net_lost", "cmd_new", "cmd_replay", "cmd_conflict",
    "cmd_invalid", "cmd_reply_fail", "env_pkts", "env_bad",
    "bulk_pkts", "bulk_bytes", "bulk_bad", "cal_applied",
    "cal_rejected", "telem_gen", "telem_sendto_ok", "telem_sendto_fail",
    "imu_who_am_i", "imu_drdy_timeout", "imu_spi_timeout", "imu_fail_run", "if_out",
    "tx_report_in", "txq_ok", "txq_full", "txq_reserve",
    "tx_bad_len", "tx_lines", "tx_uart_stall", "telem_over10",
    "telem_miss", "fault_raised",
    "nb_get_fail", "sendto_no_buf", "ipq_event_lost", "arp_tx_dropped",
    "arp_delayed", "arp_delayed_full", "arp_expired", "ip_evt_rx",
    "ip_evt_tx", "ip_evt_arp_timer", "ip_evt_other",
)

INDEX = {name: i for i, name in enumerate(COUNTER_NAMES)}


def parse_header(path):
    """app_stats.h içindeki eStatId_t sırasını okur (kartla uyum kontrolü için)."""
    with open(path, encoding="utf-8") as f:
        src = f.read()
    body = src[src.index("typedef enum eSTAT_ID"):src.index("} eStatId_t;")]
    names = [n for n in re.findall(r"^\s*(eStat\w+)", body, re.M) if n != "eStatCount"]
    return tuple(re.sub(r"(?<!^)(?=[A-Z])", "_", n[5:]).lower() for n in names)


if __name__ == "__main__":
    import sys
    hdr = parse_header(sys.argv[1] if len(sys.argv) > 1 else "../App/Inc/app_stats.h")
    print("aynı" if hdr == COUNTER_NAMES else "FARKLI: stats_ids.py güncellenmeli", len(hdr))
    sys.exit(0 if hdr == COUNTER_NAMES else 1)
