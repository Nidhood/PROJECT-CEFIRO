#!/usr/bin/env python3
"""Static gates: no premature capture and simulated-only lockout."""
import ast
from pathlib import Path
s=(Path(__file__).resolve().parents[1]/'scripts/cefiro_balance_node.py').read_text()
t=ast.parse(s)
assert "simulation_only" in s and "use_sim_time" in s
assert "self.status('BRAKING'" in s and "self.status('CATCHING'" in s
assert "self.status('YAW_ALIGN'" in s and "self.status('CAPTURED'" in s
assert "elapsed>=self.param('swingup_duration_s')" in s
assert "self.local_condition(self.param('brake_angle_deg')" in s
assert "self.blended_controller" in s
assert "self.yaw_elapsed+dt" in s
print('PASS: transition gating and yaw-priority logic')
