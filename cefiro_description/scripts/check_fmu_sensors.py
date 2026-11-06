#!/usr/bin/env python3
"""Check FMU mount and chip STL geometry against the source frame STL."""

from pathlib import Path
import struct
import yaml

ROOT = Path(__file__).resolve().parents[1]
SENSORS = ROOT.parent / 'cefiro_sensors'


def bounds_stl(path):
    data = path.read_bytes()
    facets = struct.unpack_from('<I', data, 80)[0]
    assert len(data) == 84 + facets * 50, f'Expected binary STL: {path}'
    lo = [float('inf')] * 3
    hi = [float('-inf')] * 3
    for i in range(facets):
        values = struct.unpack_from('<12f', data, 84 + i * 50)
        for j in (3, 6, 9):
            for k in range(3):
                lo[k] = min(lo[k], values[j + k])
                hi[k] = max(hi[k], values[j + k])
    return lo, hi


def main():
    yaml_path = ROOT / 'config' / 'cefiro_params.yaml'
    conf = yaml.safe_load(yaml_path.read_text())['cefiro']
    assert conf['metadata']['drone_mass_measured_g'] == 1627.0
    total = sum(conf['links'][k]['mass_g'] for k in ['drone_body_link'] +
                [f'prop_{i}_link' for i in range(1, 5)])
    assert abs(total - 1627.0) < 1e-6, f'Drone mass changed: {total}'

    fmu = conf['sensors']['fmu_mount']
    assert fmu['parent'] == 'drone_body_link'
    xyz = fmu['origin_mm']
    assert abs(xyz[0] - 49.0) < .01 and abs(xyz[1] - 35.5) < .01
    assert abs(xyz[2] - 54.1014) < .01

    specs = {
        'imu_icm42670p': SENSORS / 'urdf/imu_icm42670p/meshes/base_link.stl',
        'magnetometer_mmc5983ma': SENSORS / 'urdf/magnetometer_mmc5983ma/meshes/base_link.stl',
        'bmp581': SENSORS / 'urdf/bmp581/meshes/visuals/base_link.stl',
    }
    for name, path in specs.items():
        lo, hi = bounds_stl(path)
        offset = conf['sensors'][name]['offset_mm']
        assert conf['sensors'][name]['rpy'] == [0.0, 0.0, 0.0]
        low = [xyz[k] + offset[k] + 1000.0 * lo[k] for k in range(3)]
        high = [xyz[k] + offset[k] + 1000.0 * hi[k] for k in range(3)]
        assert low[0] > 29.0 and high[0] < 69.0, (name, low, high)
        assert low[1] > -14.5 and high[1] < 85.5, (name, low, high)
        assert low[2] >= xyz[2], (name, 'chip intersects FMU top', low)
        print(f'{name}: xyz mm min={[round(v, 3) for v in low]} '
              f'max={[round(v, 3) for v in high]}')
    print(f'PASS: FMU plane z={xyz[2]} mm; 3 sensors inside footprint; total drone mass {total:.3f} g')


if __name__ == '__main__':
    main()
