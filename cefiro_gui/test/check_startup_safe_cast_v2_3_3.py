#!/usr/bin/env python3
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
src = (root/'src/main_window.cpp').read_text()
bridge = (root/'src/ros_bridge.cpp').read_text()
assert 'findChildren<AresPage*>()' not in re.sub(r'//.*', '', src)
assert 'findChildren<AresPanel*>()' not in re.sub(r'//.*', '', src)
assert 'findChildren<ScopePlot*>()' not in re.sub(r'//.*', '', src)
assert 'dynamic_cast<AresPage*>' in src
assert 'dynamic_cast<AresPanel*>' in src
assert 'dynamic_cast<ScopePlot*>' in src
assert 'std::from_chars(' in bridge
assert 'std::stod(' not in re.sub(r'//.*', '', bridge)
assert 'Theme widget types verified' in src
print('PASS: safe Qt subclass cast and locale-independent yaw')
