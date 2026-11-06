#!/usr/bin/env python3
"""Offline small-angle mapping and slew tests; no ROS 2 dependencies."""
import ast
import math
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation

root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'cefiro_control/scripts'))
from cefiro_multibody import CefiroMultibody,wrap_delta
src=root/'cefiro_control/scripts/cefiro_balance_node.py'
tree=ast.parse(src.read_text())
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='BalanceNode')
methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('update_filtered_reference','on_ref')]
assert len(methods)==2
class Control:
    def __init__(self):
        self.model=CefiroMultibody(root/'cefiro_description/config/cefiro_params.yaml')
        self.nominal_body_rotation=self.model.kinematics(np.zeros(3))[0]['drone_body_link'][0]
        self.reference=np.zeros(3)
        self.last_reference=None
        self.filtered_q_ref=np.zeros(3)
        self.filtered_q_ref_velocity=np.zeros(3)
        self.last_requested_q_ref=np.zeros(3)
        self.params={'reference_limits_deg':[5.,5.,10.], 'reference_slew_deg_s':[2.,2.,3.],
                     'reference_accel_deg_s2':[3.,3.,5.]}
        self.fault=None
    def param(self,k):return self.params[k]
    def abort(self,s):self.fault=s

ns={'np':np,'math':math,'time':__import__('time'),'Rotation':Rotation,'wrap_delta':wrap_delta}
for method in methods:
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(src),'exec'),ns)
Control.update_filtered_reference=ns['update_filtered_reference']
Control.on_ref=ns['on_ref']

for cmd,axis,expected in [([5.,0.,0.],1,-5.),([0.,5.,0.],2,5.),([0.,0.,10.],0,-10.)]:
    ctl=Control()
    v=np.radians(cmd)
    ctl.on_ref(SimpleNamespace(vector=SimpleNamespace(x=v[0],y=v[1],z=v[2])))
    assert ctl.fault is None
    last=np.zeros(3)
    for i in range(350):
        q=ctl.update_filtered_reference(.02)
        assert q is not None and ctl.fault is None,ctl.fault
        assert np.max(np.abs(np.degrees(q-last))) <= 0.061
        last=q.copy()
    deg=np.degrees(q)
    print('body:',cmd,'=> final joints [yaw,pitch,roll]:',deg.round(3))
    assert abs(deg[axis]-expected)<0.12,(cmd,deg)
    assert np.max(np.abs(deg))<11.
    # zero must return to equilibrium smoothly (no automatic recapture)
    ctl.on_ref(SimpleNamespace(vector=SimpleNamespace(x=0.,y=0.,z=0.)))
    for i in range(350):q=ctl.update_filtered_reference(.02)
    assert np.linalg.norm(np.degrees(q))<.12

ctl=Control()
ctl.on_ref(SimpleNamespace(vector=SimpleNamespace(x=math.radians(60),y=math.radians(-60),z=math.radians(200))))
assert np.allclose(np.degrees(ctl.reference),[5.,-5.,10.])
print('PASS reference mapping, ramp, safe clamp, return to zero')
