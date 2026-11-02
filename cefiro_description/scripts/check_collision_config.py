#!/usr/bin/env python3
"""Validate primitive collision settings without ROS dependencies (PyYAML required)."""
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
config = yaml.safe_load((ROOT / 'config' / 'cefiro_params.yaml').read_text())['cefiro']
links = config['links']
assert len(links) == 9
assert abs(sum(links[k]['mass_g'] for k in ['drone_body_link',
    'prop_1_link', 'prop_2_link', 'prop_3_link', 'prop_4_link']) - 1627.0) < 1e-5
assert config['metadata']['rotor_spin'] == ['CW', 'CCW', 'CW', 'CCW']
narya_parts = links['narya_ring']['collision_parts']
assert len(narya_parts) == 6, 'Expected 2 diagonal beams + 2 uprights + crossbar + hub'
assert sorted(p['name'] for p in narya_parts if 'diagonal' in p['name']) == [
    'continuous_diagonal_negative', 'continuous_diagonal_positive']
all_parts = 0
for link_name, link in links.items():
    assert (ROOT / 'meshes' / link['mesh']).is_file(), link_name
    names = set()
    for part in link['collision_parts']:
        assert part['name'] not in names
        names.add(part['name'])
        assert len(part['origin_mm']) == 3 and len(part['origin_rpy']) == 3
        if part['shape'] == 'box':
            assert len(part['dimensions_mm']) == 3
            assert all(v > 0 for v in part['dimensions_mm'])
        elif part['shape'] == 'cylinder':
            assert part['radius_mm'] > 0 and part['length_mm'] > 0
        else:
            raise ValueError(f'Unsupported collision shape {part["shape"]}')
        all_parts += 1
joints = config['joints']
assert [(joints[k]['parent'], joints[k]['child']) for k in ('yaw','pitch','roll','gyro_to_frame')] == [
    ('base_link','narya_ring'), ('narya_ring','vilya_ring'),
    ('vilya_ring','nenya_ring'), ('nenya_ring','drone_body_link')]
assert all('mesh' not in part['shape'] for l in links.values() for part in l['collision_parts'])
assert all_parts == 44, f'Unexpected total primitive count: {all_parts}'
print(f'OK: {len(links)} links, {len(joints)} joints, {all_parts} primitive collisions, no collision meshes.')
