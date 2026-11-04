#!/usr/bin/env python3
import ast
import pathlib
import subprocess
import xml.etree.ElementTree as ET
import yaml

root = pathlib.Path(__file__).resolve().parents[1]
for p in (root/'launch').glob('*.py'):
    ast.parse(p.read_text())
ET.parse(root/'package.xml')
resources = ET.parse(root/'resources/resources.qrc').getroot()
for f in resources.findall('.//file'):
    assert (root/'resources'/f.text).is_file(), f.text
conf = yaml.safe_load((root/'config/gui_config.yaml').read_text())
assert conf['cefiro_hmi']['ros__parameters']['imu_topic'] == '/imu/data'
assert len(list((root/'resources/theme').glob('*.qss'))) == 6
headers = list((root/'include').rglob('*.hpp'))
cpps = list((root/'src').glob('*.cpp'))
assert len(headers) >= 5 and len(cpps) >= 5
assert all(p.stat().st_size > 100 for p in headers + cpps)
print(f'PACKAGE CHECKS PASSED: {len(headers)} headers, {len(cpps)} source files, six Qt themes')
