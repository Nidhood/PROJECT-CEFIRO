#!/usr/bin/env python3
"""Model-based LQR / gravity / allocation smoke checks (no ROS dependency)."""
import sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'cefiro_control/scripts'))
from cefiro_multibody import CefiroMultibody,lqr_gains,allocate_motors
model=CefiroMultibody(ROOT/'cefiro_description/config/cefiro_params.yaml')
M,g=model.mass_gravity(np.zeros(3))
assert np.min(np.linalg.eigvalsh(M))>0
K=lqr_gains(M,np.array([.05,.05,.05]),[14.,12.,12.],[70.,65.,65.],[.07,.20,.20])
assert K.shape==(3,6) and np.all(np.isfinite(K))
q=np.array([0.,0.,-.3])
_,v=model.mass_gravity(q)
eps=1.e-5
num=(model.potential(q+np.array([0,0,eps]))-model.potential(q-np.array([0,0,eps])))/(2*eps)
assert abs(num-v[-1])<1e-4
G=model.motor_effectiveness(q,1.416343e-7,5.396267e-9)
for w in ((1,1,1),(.35,1,1)):
    rpm,err,act=allocate_motors(G,v,4700,5400,axis_weights=w)
    assert np.all(np.isfinite(rpm)) and np.all(rpm>=0) and np.max(rpm)<=5400
print('PASS: URDF gravity, LQR, allocation')
