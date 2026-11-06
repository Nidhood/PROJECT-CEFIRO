#!/usr/bin/env python3
"""Offline regression of CEFIRO handoff using 3DOF URDF dynamics.

This is not a Gazebo test; it models motor lag and bounded RPM only.
"""
import ast
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'cefiro_control/scripts'))
from cefiro_multibody import CefiroMultibody, wrap_delta, fifth_order, lqr_gains, allocate_motors


class SimClock:
    t=0.0
    def monotonic(self):return self.t


class Logger:
    def info(self,s):pass
    def warn(self,s):pass
    def error(self,s):print('ERROR:',s)


class Param:
    def __init__(self,value):self.value=value


class Pub:
    def __init__(self):self.last=None;self.count=0
    def publish(self,msg):self.last=msg;self.count+=1


class Msg:
    def __init__(self,data=None,armed=False,measured_rpm=None,watchdog_ok=True):
        self.data=data
        self.armed=armed
        self.measured_rpm=[0.0]*4 if measured_rpm is None else measured_rpm
        self.watchdog_ok=watchdog_ok


class Node:
    def __init__(self,*args):
        self.parameters={'use_sim_time':True}
        self.pubs={}
    def declare_parameter(self,k,v):
        if k=='model_yaml':v=str(ROOT/'cefiro_description/config/cefiro_params.yaml')
        self.parameters[k]=v
    def get_parameter(self,k):return Param(self.parameters[k])
    def create_publisher(self,cls,topic,qos):
        p=Pub();self.pubs[topic]=p;return p
    def create_subscription(self,*args):return None
    def create_timer(self,*args):return None
    def get_logger(self):return Logger()

clock=SimClock()
script=ROOT/'cefiro_control/scripts/cefiro_balance_node.py'
parsed=ast.parse(script.read_text())
cls=next(n for n in parsed.body if isinstance(n,ast.ClassDef) and n.name=='BalanceNode')
namespace=dict(Node=Node,np=np,math=math,json=json,time=clock,Path=Path,Rotation=Rotation,
               CefiroMultibody=CefiroMultibody,wrap_delta=wrap_delta,fifth_order=fifth_order,
               lqr_gains=lqr_gains,allocate_motors=allocate_motors,
               Imu=Msg,MagneticField=Msg,FluidPressure=Msg,Vector3Stamped=Msg,
               MotorRpm=Msg,MotorTelemetry=Msg,Bool=Msg,String=Msg,qos_profile_sensor_data=None)
exec(compile(ast.Module(body=[cls],type_ignores=[]),str(script),'exec'),namespace)
controller=namespace['BalanceNode']()
controller.enabled=True
controller.mode='ATTITUDE'
controller.strategy='SWING-UP + LQR (SIM)'

# Initial coordinates fitted from observed initial condition in 2026-11-06 CSV.
q=np.radians([81.1,-0.4,-178.7])
qd=np.zeros(3)
rpm=np.zeros(4)
armed=False
states=[]
max_rates=0.
max_error=0.
dt=0.02
model=controller.model
captured_since=None
for k in range(int(48/dt)):
    clock.t+=dt
    poses,_,axes=model.kinematics(q)
    R=poses['drone_body_link'][0]
    body_rates=R.T@axes.T@qd
    imu=SimpleNamespace(
        orientation=SimpleNamespace(x=Rotation.from_matrix(R).as_quat()[0],
                                    y=Rotation.from_matrix(R).as_quat()[1],
                                    z=Rotation.from_matrix(R).as_quat()[2],
                                    w=Rotation.from_matrix(R).as_quat()[3]),
        orientation_covariance=[0.0]*9,
        angular_velocity=SimpleNamespace(x=body_rates[0],y=body_rates[1],z=body_rates[2]))
    controller.on_imu(imu)
    controller.last_mag=clock.t
    controller.last_reference=clock.t
    controller.on_telemetry(Msg(armed=armed,measured_rpm=rpm.tolist()))
    controller.update()
    if controller.arm_pub.last is not None:
        armed=controller.arm_pub.last.data
    if controller.motor_pub.last is not None:
        cmd=np.asarray(controller.motor_pub.last.data,float)
    else:cmd=np.zeros(4)
    if not armed:cmd*=0.0
    # First-order RPM actuator lag only; no electronics or contact modeling.
    rpm+=(cmd-rpm)*min(1.,dt/.065)
    rho=1.225;D=0.254
    G=model.motor_effectiveness(q,0.10*rho*D**4/3600.,0.015*rho*D**5/3600.)
    achieved=G@(rpm**2)
    M,g=model.mass_gravity(q)
    c=model.coriolis(q,qd)
    qdd=np.linalg.solve(M,achieved-c-g-controller.damping*qd)
    qd+=dt*qdd
    q+=dt*qd
    if not np.all(np.isfinite(q)):raise SystemExit('FAIL NONFINITE')
    if abs(qd).max()>14:raise SystemExit('FAIL EXTREME_RATE')
    states.append(controller.state)
    if controller.state=='CATCHING' and 'CATCHING' not in states[:-1]:
        assert clock.t-controller.swing_time>=controller.param('swingup_duration_s')
        assert max(abs(np.degrees(wrap_delta(controller.q_est[1:]))))<controller.param('brake_angle_deg')
        assert max(abs(np.degrees(controller.qd_est[1:])))<controller.param('brake_rate_deg_s')
    max_rates=max(max_rates,abs(qd).max())
    max_error=max(max_error,abs(controller.last_tau-controller.last_achieved).max())
    if k%250==0:
        print(f'{clock.t:5.1f}s {controller.state:12s} q={np.degrees(wrap_delta(q)).round(1)} '
            f'qd={np.degrees(qd).round(1)} residual={np.linalg.norm(controller.last_tau-controller.last_achieved):.3f}')
    if controller.state=='CAPTURED':
        if captured_since is None:captured_since=clock.t
        elif clock.t-captured_since>2.0:break
    if controller.state.startswith('FAULT_'):
        break

transitions=[]
for st in states:
    if not transitions or transitions[-1]!=st:transitions.append(st)
print('TRANSITIONS',transitions)
print('FINAL',controller.state,'q',np.degrees(wrap_delta(q)).round(2),'rate',np.degrees(qd).round(2),
    'max_speed_deg_s',round(math.degrees(max_rates),1),'max_error',round(max_error,3))
assert 'BRAKING' in transitions, 'missing braking state'
assert 'CATCHING' not in transitions[:transitions.index('BRAKING')], 'premature LQR switch'
assert not any(s.startswith('FAULT_') for s in states), controller.state
assert controller.state=='CAPTURED', controller.state
assert max(abs(np.degrees(wrap_delta(q))))<10
assert max(abs(np.degrees(qd)))<12
print('PASS: model-based 3DOF handoff / rate-limited yaw / local equilibrium')
