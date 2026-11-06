#!/usr/bin/env python3
"""Independent tests of multibody geometry and force projection (no ROS)."""
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'cefiro_control/scripts'))
from cefiro_multibody import CefiroMultibody,wrap_delta,allocate_motors
model=CefiroMultibody(ROOT/'cefiro_description/config/cefiro_params.yaml')
rng=np.random.default_rng(20261106)
for _ in range(16):
    q=rng.uniform(-1.2,1.2,3)
    M,g=model.mass_gravity(q)
    assert np.linalg.eigvalsh(M).min()>0
    dq=1.e-5
    gn=np.array([(model.potential(q+np.eye(3)[k]*dq)-model.potential(q-np.eye(3)[k]*dq))/(2*dq) for k in range(3)])
    assert np.allclose(g,gn,atol=1e-4)
    R=model.kinematics(q)[0]['drone_body_link'][0]
    qest,res,cond=model.estimate_joints(R,q+.01)
    assert res<1e-4 and np.linalg.norm(wrap_delta(q-qest))<1e-3
rho=1.225;D=.254
kf=.10*rho*D**4/3600
km=.015*rho*D**5/3600
for a in (-np.pi,0,np.pi):
    q=np.array([0.,0.,a]);G=model.motor_effectiveness(q,kf,km)
    torque=model.mass_gravity(q)[1]
    rpm,res,ach=allocate_motors(G,torque,4700,5400,axis_weights=(.35,1,1))
    assert res<.1 and np.max(rpm)<=5400
print('PASS: URDF mass, gravity, inverse kinematics, motor allocation')
