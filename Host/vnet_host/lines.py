"""UART satırları (tasarım 2.1, 13.4, 13.5).

host -> kart:  FRAME <hex> | FRAME 0 <hex> | FRAME 1 <hex+FCS> | HELLO | STATE | HOLD <ms> | TXDELAY <ms>
kart -> host:  FRAME 1 <hex+FCS> | {json}
Her satır '\n' ile biter. Kart '\r'yi yok sayar.
"""

from dataclasses import dataclass
import json

from .crc import fcs_bytes, fcs_ok

LINE_MAX = 540                 # kartın satır sınırı (line_rx)


def frame_line(frame: bytes, fcs_flag=1) -> bytes:
    """fcs_flag: None -> 'FRAME <hex>', 0 -> 'FRAME 0 <hex>', 1 -> 'FRAME 1 <hex><FCS>'."""
    if fcs_flag is None:
        return b"FRAME " + frame.hex().upper().encode() + b"\n"
    if fcs_flag == 0:
        return b"FRAME 0 " + frame.hex().upper().encode() + b"\n"
    return b"FRAME 1 " + (frame + fcs_bytes(frame)).hex().upper().encode() + b"\n"


def console_line(cmd: str) -> bytes:
    return cmd.encode("ascii") + b"\n"


@dataclass
class BoardLine:
    """Karttan gelen bir satır. kind: 'frame', 'json' ya da 'bad'."""
    kind: str
    text: str
    frame: bytes = b""            # FCS hariç
    report: dict = None
    error: str = ""
    rx_time: float = 0.0


def parse_board_line(text: str, rx_time: float = 0.0) -> BoardLine:
    text = text.rstrip("\r")
    if text.startswith("{"):
        try:
            obj = json.loads(text)
        except ValueError:
            return BoardLine("bad", text, error="json", rx_time=rx_time)
        if not isinstance(obj, dict) or "type" not in obj:
            return BoardLine("bad", text, error="json_type", rx_time=rx_time)
        return BoardLine("json", text, report=obj, rx_time=rx_time)
    if text.startswith("FRAME 1 "):
        hexpart = text[8:]
        try:
            raw = bytes.fromhex(hexpart)
        except ValueError:
            return BoardLine("bad", text, error="hex", rx_time=rx_time)
        if len(raw) < 64:
            return BoardLine("bad", text, error="frame_length", rx_time=rx_time)
        if not fcs_ok(raw):
            return BoardLine("bad", text, error="fcs", rx_time=rx_time)
        return BoardLine("frame", text, frame=raw[:-4], rx_time=rx_time)
    return BoardLine("bad", text, error="unknown", rx_time=rx_time)
