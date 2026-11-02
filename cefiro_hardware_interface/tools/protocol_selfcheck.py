#!/usr/bin/env python3

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cefiro_hardware_interface.protocol import (encode_frame, decode_frame, crc16_ccitt_false,
                                               ProtocolError, LineBuffer)
from cefiro_hardware_interface.legacy import parse_legacy_line, ImuSample, MagSample, BaroSample, SensorError

assert crc16_ccitt_false(b'123456789') == 0x29B1
packet = encode_frame(15, 'IMU', ('-0.196', '-0.402', '9.814', '-0.107', '-0.214', '0.015', '28.306'))
frame = decode_frame(packet)
assert frame.kind == 'IMU' and frame.sequence == 15 and len(frame.payload) == 7
try:
    decode_frame(packet.replace(b'IMU', b'XYZ'))
    raise AssertionError('CRC tampering accepted')
except ProtocolError:
    pass
buf = LineBuffer()
assert buf.feed(packet[:9]) == []
assert buf.feed(packet[9:]) == [packet.rstrip(b'\n')]
assert isinstance(parse_legacy_line('IMU  | A[m/s2] X=-0.196 Y=-0.402 Z=9.814 | G[deg/s] X=-0.107 Y=-0.214 Z=0.015 | T=28.306 C'), ImuSample)
assert isinstance(parse_legacy_line('MAG  | B[uT] X=20.123 Y=-4.100 Z=47.010'), MagSample)
assert isinstance(parse_legacy_line('BARO | P=742.054 hPa | T=27.650 C | Alt_ISA=2551.345 m | P_RAW=742.053 hPa | Z_REL=-1.083 m'), BaroSample)
assert isinstance(parse_legacy_line('MAG  | ERROR: PRODUCT_ID/SPI4 or data not ready'), SensorError)
assert parse_legacy_line('IMU  | A[m/s2] X=N/A Y=0 Z=9 | G[deg/s] X=0 Y=0 Z=0 | T=25 C') is None
print('PASS: CRC16, encoder/decoder, segmented serial framing, exact main.c IMU/MAG/BARO parsing, error handling')
