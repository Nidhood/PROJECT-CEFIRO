from __future__ import annotations
from dataclasses import dataclass
import math
import re
from typing import Optional, Tuple, Union

NUMBER = r'([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)'
IMU_RE = re.compile(
    r'^IMU\s*\|\s*A\[m/s2\]\s*X=' + NUMBER + r'\s+Y=' + NUMBER + r'\s+Z=' + NUMBER +
    r'\s*\|\s*G\[deg/s\]\s*X=' + NUMBER + r'\s+Y=' + NUMBER + r'\s+Z=' + NUMBER +
    r'\s*\|\s*T=' + NUMBER + r'\s*C\s*$')
MAG_RE = re.compile(r'^MAG\s*\|\s*B\[uT\]\s*X=' + NUMBER + r'\s+Y=' + NUMBER + r'\s+Z=' + NUMBER + r'\s*$')
BARO_RE = re.compile(
    r'^BARO\s*\|\s*P=' + NUMBER + r'\s+hPa\s*\|\s*T=' + NUMBER + r'\s+C\s*\|\s*Alt_ISA=' + NUMBER +
    r'\s+m\s*\|\s*P_RAW=' + NUMBER + r'\s+hPa\s*\|\s*Z_REL=' + NUMBER + r'\s+m\s*$')


@dataclass(frozen=True)
class ImuSample:
    acceleration: Tuple[float, float, float]
    gyro_dps: Tuple[float, float, float]
    temperature_c: float


@dataclass(frozen=True)
class MagSample:
    magnetic_ut: Tuple[float, float, float]


@dataclass(frozen=True)
class BaroSample:
    pressure_hpa: float
    temperature_c: float
    altitude_isa_m: float
    raw_pressure_hpa: float
    relative_altitude_m: float


@dataclass(frozen=True)
class SensorError:
    sensor: str
    detail: str


def parse_legacy_line(line: str) -> Optional[Union[ImuSample, MagSample, BaroSample, SensorError]]:
    line = line.strip()
    if re.match(r'^(IMU|MAG|BARO)\s*\|\s*(ERROR|CALIBRATING):', line):
        head, message = line.split('|', 1)
        return SensorError(head.strip(), message.strip())
    for regex, message_type in ((IMU_RE, 'IMU'), (MAG_RE, 'MAG'), (BARO_RE, 'BARO')):
        match = regex.fullmatch(line)
        if match is None:
            continue
        values = tuple(float(group) for group in match.groups())
        if not all(math.isfinite(v) for v in values):
            return None
        if message_type == 'IMU':
            return ImuSample(values[:3], values[3:6], values[6])
        if message_type == 'MAG':
            return MagSample(values)
        if values[0] <= 0 or values[3] <= 0:
            return None
        return BaroSample(*values)
    return None
