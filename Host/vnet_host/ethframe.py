"""Ethernet / ARP / IPv4 / UDP / ICMP: üretme ve doğrulama (tasarım 13.2, 14, 15.4).

Kartın vnet_if.c kontrolleriyle aynı sıra: EtherType -> hedef MAC -> IPv4 başlık
checksum -> UDP/ICMP checksum. Host, karttan gelen her frame'i aynı biçimde doğrular.
"""

from dataclasses import dataclass, field
import ipaddress
import struct

ETH_HDR = 14
ETH_MIN = 60                   # FCS hariç en kısa frame
ETH_MAX = 256                  # kartın kabul ettiği en uzun (FCS hariç)
ETHERTYPE_IPV4 = 0x0800
ETHERTYPE_ARP = 0x0806
IP_PROTO_ICMP = 1
IP_PROTO_UDP = 17
BROADCAST = b"\xff" * 6

HOST_MAC = bytes.fromhex("020000000001")
HOST_IP = "192.168.50.1"
BOARD_IP = "192.168.50.2"
HOST_PORT = 6000


def mac_str(mac: bytes) -> str:
    return ":".join("%02X" % b for b in mac)


def ip_bytes(ip) -> bytes:
    return ipaddress.IPv4Address(ip).packed if isinstance(ip, str) else bytes(ip)


def ip_str(b: bytes) -> str:
    return str(ipaddress.IPv4Address(bytes(b)))


def checksum16(data: bytes) -> int:
    """İnternet checksum'ı (RFC 1071): 16-bit bire tümleyen toplamın tümleyeni."""
    if len(data) & 1:
        data = data + b"\x00"
    s = sum(struct.unpack(">%dH" % (len(data) // 2), data))
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF


def _pad(frame: bytes) -> bytes:
    return frame + b"\x00" * (ETH_MIN - len(frame)) if len(frame) < ETH_MIN else frame


def eth_header(dst: bytes, src: bytes, ethertype: int) -> bytes:
    return bytes(dst) + bytes(src) + struct.pack(">H", ethertype)


def ipv4_header(src, dst, proto: int, payload_len: int, ident: int = 1, ttl: int = 64,
                bad_checksum: bool = False) -> bytes:
    hdr = struct.pack(">BBHHHBBH4s4s", 0x45, 0, 20 + payload_len, ident & 0xFFFF, 0, ttl, proto, 0,
                      ip_bytes(src), ip_bytes(dst))
    cs = checksum16(hdr)
    if bad_checksum:
        cs ^= 0x00FF
    return hdr[:10] + struct.pack(">H", cs) + hdr[12:]


def udp_checksum(src, dst, udp: bytes) -> int:
    pseudo = ip_bytes(src) + ip_bytes(dst) + struct.pack(">BBH", 0, IP_PROTO_UDP, len(udp))
    cs = checksum16(pseudo + udp)
    return cs if cs != 0 else 0xFFFF


def build_udp_frame(dst_mac: bytes, src_mac: bytes, src_ip, dst_ip, sport: int, dport: int,
                    payload: bytes, ident: int = 1, ttl: int = 64,
                    bad_ip_checksum: bool = False, bad_udp_checksum: bool = False) -> bytes:
    udp = struct.pack(">HHHH", sport, dport, 8 + len(payload), 0) + payload
    cs = udp_checksum(src_ip, dst_ip, udp)
    if bad_udp_checksum:
        cs ^= 0x5555
    udp = udp[:6] + struct.pack(">H", cs) + udp[8:]
    ip = ipv4_header(src_ip, dst_ip, IP_PROTO_UDP, len(udp), ident, ttl, bad_ip_checksum)
    return _pad(eth_header(dst_mac, src_mac, ETHERTYPE_IPV4) + ip + udp)


def build_arp(op: int, sender_mac: bytes, sender_ip, target_mac: bytes, target_ip,
              eth_dst: bytes = None) -> bytes:
    """op 1 = istek, 2 = cevap."""
    arp = struct.pack(">HHBBH6s4s6s4s", 1, ETHERTYPE_IPV4, 6, 4, op, sender_mac, ip_bytes(sender_ip),
                      target_mac, ip_bytes(target_ip))
    dst = eth_dst if eth_dst is not None else (BROADCAST if op == 1 else target_mac)
    return _pad(eth_header(dst, sender_mac, ETHERTYPE_ARP) + arp)


def build_icmp_echo(dst_mac: bytes, src_mac: bytes, src_ip, dst_ip, ident: int, seq: int,
                    data: bytes = b"vnet-host-ping", ip_ident: int = 1) -> bytes:
    icmp = struct.pack(">BBHHH", 8, 0, 0, ident, seq) + data
    icmp = icmp[:2] + struct.pack(">H", checksum16(icmp)) + icmp[4:]
    ip = ipv4_header(src_ip, dst_ip, IP_PROTO_ICMP, len(icmp), ip_ident)
    return _pad(eth_header(dst_mac, src_mac, ETHERTYPE_IPV4) + ip + icmp)


@dataclass
class Frame:
    """Çözülmüş frame. errors boşsa doğrulamadan geçti."""
    raw: bytes
    dst: bytes = b""
    src: bytes = b""
    ethertype: int = 0
    errors: list = field(default_factory=list)
    # ARP
    arp_op: int = 0
    arp_sender_mac: bytes = b""
    arp_sender_ip: str = ""
    arp_target_ip: str = ""
    # IPv4
    ip_src: str = ""
    ip_dst: str = ""
    ip_proto: int = 0
    ip_ident: int = 0
    ttl: int = 0
    # UDP
    sport: int = 0
    dport: int = 0
    udp_payload: bytes = b""
    # ICMP
    icmp_type: int = -1
    icmp_ident: int = 0
    icmp_seq: int = 0

    @property
    def ok(self) -> bool:
        return not self.errors

    @property
    def kind(self) -> str:
        if self.ethertype == ETHERTYPE_ARP:
            return "ARP"
        if self.ip_proto == IP_PROTO_UDP:
            return "UDP"
        if self.ip_proto == IP_PROTO_ICMP:
            return "ICMP"
        return "0x%04X" % self.ethertype


def parse_frame(frame: bytes, my_mac: bytes = None) -> Frame:
    """frame: FCS hariç. my_mac verilirse hedef MAC'in bizim ya da yayın olması da istenir."""
    f = Frame(raw=bytes(frame))
    if len(frame) < ETH_MIN:
        f.errors.append("frame_length")
        if len(frame) < ETH_HDR:
            return f
    f.dst, f.src = frame[0:6], frame[6:12]
    f.ethertype = struct.unpack(">H", frame[12:14])[0]
    if f.ethertype not in (ETHERTYPE_IPV4, ETHERTYPE_ARP):
        f.errors.append("frame_ethertype")
        return f
    if my_mac is not None and f.dst not in (bytes(my_mac), BROADCAST):
        f.errors.append("frame_dst_mac")
    body = frame[ETH_HDR:]
    if f.ethertype == ETHERTYPE_ARP:
        if len(body) < 28:
            f.errors.append("arp_short")
            return f
        htype, ptype, hlen, plen, op, sha, spa, tha, tpa = struct.unpack(">HHBBH6s4s6s4s", body[:28])
        if (htype, ptype, hlen, plen) != (1, ETHERTYPE_IPV4, 6, 4):
            f.errors.append("arp_format")
        f.arp_op, f.arp_sender_mac = op, sha
        f.arp_sender_ip, f.arp_target_ip = ip_str(spa), ip_str(tpa)
        return f
    # IPv4
    if len(body) < 20 or (body[0] >> 4) != 4 or (body[0] & 0x0F) < 5:
        f.errors.append("ip_csum")          # kartla aynı ad: kullanılamaz başlık
        return f
    ihl = (body[0] & 0x0F) * 4
    total = struct.unpack(">H", body[2:4])[0]
    if len(body) < ihl or total < ihl or total > len(body):
        f.errors.append("ip_length")
        return f
    if checksum16(body[:ihl]) != 0:
        f.errors.append("ip_csum")
    f.ip_ident = struct.unpack(">H", body[4:6])[0]
    f.ttl, f.ip_proto = body[8], body[9]
    f.ip_src, f.ip_dst = ip_str(body[12:16]), ip_str(body[16:20])
    l4 = body[ihl:total]
    if f.ip_proto == IP_PROTO_UDP:
        if len(l4) < 8:
            f.errors.append("udp_length")
            return f
        f.sport, f.dport, ulen, ucs = struct.unpack(">HHHH", l4[:8])
        if ulen < 8 or ulen > len(l4):
            f.errors.append("udp_length")
            return f
        udp = l4[:ulen]
        if ucs == 0:
            f.errors.append("udp_csum")       # kart (stack) her zaman checksum yazar
        else:
            pseudo = body[12:20] + struct.pack(">BBH", 0, IP_PROTO_UDP, ulen)
            if checksum16(pseudo + udp) != 0:
                f.errors.append("udp_csum")
        f.udp_payload = bytes(udp[8:])
    elif f.ip_proto == IP_PROTO_ICMP:
        if len(l4) < 8:
            f.errors.append("icmp_length")
            return f
        if checksum16(l4) != 0:
            f.errors.append("udp_csum")       # kartta ICMP de bu sayaçta (V1)
        f.icmp_type = l4[0]
        f.icmp_ident, f.icmp_seq = struct.unpack(">HH", l4[4:8])
    return f
