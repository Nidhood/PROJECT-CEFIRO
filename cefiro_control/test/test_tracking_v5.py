#!/usr/bin/env python3
"""Offline regression for CAPTURED reference trim. No ROS 2 or Gazebo needed."""
from pathlib import Path
import ast, math, numpy as np

source=Path(__file__).parents[1]/'scripts'/'cefiro_balance_node.py'
node=ast.parse(source.read_text(encoding='utf-8'))
klass=next(n for n in node.body if isinstance(n,ast.ClassDef) and n.name=='BalanceNode')
method=next(n for n in klass.body if isinstance(n,ast.FunctionDef) and n.name=='captured_integral_correction')
ns={'np':np,'math':math,'wrap_delta':lambda x:np.arctan2(np.sin(x),np.cos(x))}
exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),ns)

class Model:
    captured_integral_correction=ns['captured_integral_correction']
    def __init__(self):
        self.reference_integral_torque=np.zeros(3)
        self.filtered_q_ref_velocity=np.zeros(3)
        self.q_est=np.array([0.,-math.radians(2.),math.radians(1.)])
        self.command=np.ones(4)*4700.
        self.measured_rpm=np.ones(4)*4700.
        self.last_allocation_residual=0.001
        self.params={
            'reference_integral_gain_nm_rad_s':[0.12,0.45,0.45],
            'reference_integral_limit_nm':[0.055,0.12,0.12],
            'reference_integral_leak_per_s':0.015,
            'reference_integral_allocation_gate_nm':0.025,
            'reference_integral_rpm_gate':450.,
            'reference_integral_target_rate_gate_deg_s':0.8}
        self.fault=None
    def param(self,key):return self.params[key]
    def abort(self,why):self.fault=why

m=Model()
for _ in range(500):t=m.captured_integral_correction(np.zeros(3),.02)
assert m.fault is None
assert t[1]>0 and t[2]<0, t
assert np.all(np.abs(t)<=np.array(m.params['reference_integral_limit_nm'])+1e-12)
# Do not wind up when allocator cannot realize torque.
before=t.copy();m.last_allocation_residual=.12
for _ in range(100):m.captured_integral_correction(np.zeros(3),.02)
assert np.all(np.abs(m.reference_integral_torque)<=np.abs(before)+1e-12)
# While following a trajectory, integral must be held/leaked rather than collecting tracking lag.
before=m.reference_integral_torque.copy();m.last_allocation_residual=0.;m.filtered_q_ref_velocity[1]=math.radians(5.)
for _ in range(100):m.captured_integral_correction(np.zeros(3),.02)
assert abs(m.reference_integral_torque[1]) < abs(before[1])
# Direction reversal must reverse integral after sufficient stable tracking.
m.filtered_q_ref_velocity[:]=0.;m.q_est=np.array([0.,math.radians(3.),-math.radians(2.)])
for _ in range(950):m.captured_integral_correction(np.zeros(3),.02)
assert m.reference_integral_torque[1]<0 and m.reference_integral_torque[2]>0
print('PASS v5 integral trim, limiting, anti-windup gate, no moving-target windup, reversal')
