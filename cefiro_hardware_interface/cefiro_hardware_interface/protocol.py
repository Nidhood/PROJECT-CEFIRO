from __future__ import annotations
from dataclasses import dataclass
import math
import re
from typing import Optional, Tuple

MAX_FRAME_BYTES = 2048
TYPE_RE = re.compile(r"^[A-Z][A-Z0-9_]{0,31}$")


class ProtocolError(ValueError):
    """A damaged, invalid or unsupported frame."""


@dataclass(frozen=True)
class Frame:
    sequence: int
    kind: str
    payload: Tuple[str, ...]


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(sequence: int, kind: str, payload=()) -> bytes:
    if isinstance(sequence, bool) or not 0 <= sequence <= 65535:
        raise ProtocolError('sequence must be uint16')
    if not TYPE_RE.fullmatch(kind):
        raise ProtocolError('invalid message kind')
    values = []
    for value in payload:
        txt = str(value)
        if any(x in txt for x in ('\n', '\r', '|', '*', ',')):
            raise ProtocolError('invalid payload character')
        if len(txt) > 96:
            raise ProtocolError('oversized field')
        values.append(txt)
    core = f'CF1|{sequence}|{kind}|{",".join(values)}'.encode('ascii')
    result = b'@' + core + f'*{crc16_ccitt_false(core):04X}\n'.encode('ascii')
    if len(result) > MAX_FRAME_BYTES:
        raise ProtocolError('frame exceeds maximum length')
    return result


def decode_frame(line: bytes) -> Frame:
    if len(line) > MAX_FRAME_BYTES:
        raise ProtocolError('frame too large')
    try:
        txt = line.decode('ascii').strip('\r\n')
    except UnicodeDecodeError as exc:
        raise ProtocolError('non-ASCII frame') from exc
    if not txt.startswith('@') or txt.count('*') != 1:
        raise ProtocolError('invalid frame envelope')
    core, crc_txt = txt[1:].rsplit('*', 1)
    if not re.fullmatch(r'[0-9A-Fa-f]{4}', crc_txt):
        raise ProtocolError('invalid CRC notation')
    if crc16_ccitt_false(core.encode('ascii')) != int(crc_txt, 16):
        raise ProtocolError('CRC mismatch')
    parts = core.split('|', 3)
    if len(parts) != 4 or parts[0] != 'CF1' or not parts[1].isdigit():
        raise ProtocolError('invalid header')
    seq = int(parts[1])
    if seq > 65535 or not TYPE_RE.fullmatch(parts[2]):
        raise ProtocolError('invalid sequence / message kind')
    payload = tuple(parts[3].split(',')) if parts[3] else ()
    return Frame(seq, parts[2], payload)


def floats(values, expected: int, *, minimum=None, maximum=None):
    if len(values) != expected:
        raise ProtocolError(f'expected {expected} fields, got {len(values)}')
    out = []
    for value in values:
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ProtocolError('invalid floating point field') from exc
        if not math.isfinite(number):
            raise ProtocolError('NaN/Inf not allowed')
        if minimum is not None and number < minimum:
            raise ProtocolError('field below permitted range')
        if maximum is not None and number > maximum:
            raise ProtocolError('field above permitted range')
        out.append(number)
    return tuple(out)


def integers(values, expected: int, minimum=0, maximum=65535):
    if len(values) != expected:
        raise ProtocolError(f'expected {expected} integer fields')
    out = []
    for value in values:
        if not re.fullmatch(r'[+-]?\d+', value):
            raise ProtocolError('non-integer field')
        parsed = int(value)
        if not minimum <= parsed <= maximum:
            raise ProtocolError('out-of-range integer')
        out.append(parsed)
    return tuple(out)


class LineBuffer:
    """Bounded serial stream -> complete newline-terminated records."""
    def __init__(self, max_line: int = MAX_FRAME_BYTES):
        self.max_line = max_line
        self.buffer = bytearray()
        self.dropped = 0

    def clear(self):
        self.buffer.clear()

    def feed(self, chunk: bytes):
        self.buffer.extend(chunk)
        if len(self.buffer) > self.max_line * 8:
            last_break = self.buffer.rfind(b'\n')
            self.buffer = self.buffer[last_break + 1:] if last_break >= 0 else bytearray()
            self.dropped += 1
        result = []
        while True:
            at = self.buffer.find(b'\n')
            if at < 0:
                break
            line = bytes(self.buffer[:at]).rstrip(b'\r')
            del self.buffer[:at+1]
            if len(line) > self.max_line:
                self.dropped += 1
            elif line:
                result.append(line)
        return result
