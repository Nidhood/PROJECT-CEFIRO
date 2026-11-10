#!/usr/bin/env python3
"""CEFIRO 3-DOF gimbal dynamics from cefiro_description/config/cefiro_params.yaml.

All kinematic/inertial parameters are taken from the user-supplied Xacro YAML,
NOT from the free-flight quadrotor inertias. The URDF defines box/disk
approximations; the resulting dynamics remain only as accurate as these inputs.

Joint coordinates are [yaw, pitch, roll] in the actual URDF joint order.
Positions and rotations are expressed in world coordinates. Generalized forces
are conjugate to joint coordinates; NOT body-fixed roll/pitch/yaw torques.
"""
import math
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from scipy.linalg import solve_continuous_are
from scipy.optimize import lsq_linear

NAMES = ('yaw', 'pitch', 'roll')
MOVING = ('narya_ring','vilya_ring','nenya_ring','drone_body_link',
          'prop_1_link','prop_2_link','prop_3_link','prop_4_link')


def rot_rpy(rpy):
    return Rotation.from_euler('xyz', rpy).as_matrix()


def rot_axis(axis, value):
    a = np.asarray(axis, float)
    a /= np.linalg.norm(a)
    return Rotation.from_rotvec(a * value).as_matrix()


def wrap_delta(a):
    return np.arctan2(np.sin(a), np.cos(a))


class CefiroMultibody:
    def __init__(self, yaml_file, spawn_rpy=(0.,0.,1.07), gravity=9.81):
        path = Path(yaml_file)
        cfg = yaml.safe_load(path.read_text(encoding='utf-8'))['cefiro']
        self.cfg = cfg
        self.links = cfg['links']
        self.joints = cfg['joints']
        self.base_rotation = rot_rpy(spawn_rpy)
        self.gravity = float(gravity)
        self.inertia = {}
        self.masses = {}
        for name, info in self.links.items():
            mass = info['mass_g'] / 1000.
            kind = info['inertia_model']
            if kind == 'box':
                dx,dy,dz = np.asarray(info['dimensions_mm'],float)/1000.
                diag = mass/12.*np.array([dy**2+dz**2,dx**2+dz**2,dx**2+dy**2])
                tensor = np.diag(diag)
            elif kind == 'disk':
                r = info['radius_mm']/1000.
                h = info['thickness_mm']/1000.
                tensor = np.diag([mass*(3*r*r+h*h)/12.]*2+[mass*r*r/2.])
            elif kind == 'tensor':
                s=info['tensor_g_mm2'];tensor=np.array([[s['ixx'],s['ixy'],s['ixz']],
                          [s['ixy'],s['iyy'],s['iyz']],[s['ixz'],s['iyz'],s['izz']]],float)*1e-9
            else:
                raise ValueError(f'Unknown inertia model {kind}: {name}')
            self.inertia[name] = tensor
            self.masses[name] = mass
        # Motor geometry: body-frame positions from the same URDF YAML used by Gazebo.
        self.motor_positions = np.array([
            np.asarray(self.joints['motor_'+str(i)]['origin_mm'],float)/1000.
            for i in range(1,5)])
        self.motor_spin = np.array([1.,-1.,1.,-1.])
        if cfg['metadata']['rotor_spin'] != ['CW','CCW','CW','CCW']:
            raise ValueError('Rotor spin convention changed: validate reaction torque signs.')
        self.q_zero = np.zeros(3)

    def _joint(self, parent_R, parent_p, name, val):
        j=self.joints[name]
        rpy=j.get('origin_rpy',[0.,0.,0.])
        xyz=np.asarray(j['origin_mm'],float)/1000.
        Rj=parent_R@rot_rpy(rpy)
        pj=parent_p+parent_R@xyz
        axis=Rj@np.asarray(j['axis'],float)
        axis=axis/np.linalg.norm(axis)
        return Rj@rot_axis(j['axis'],val),pj,axis

    def kinematics(self, q):
        """Return body transforms, and joint origin/axis in world frame."""
        q=np.asarray(q,float)
        R=self.base_rotation.copy()
        p=np.zeros(3)
        poses={'base_link':(R,p)}
        pivots=[];axes=[]
        for idx,(j,child) in enumerate(zip(NAMES,('narya_ring','vilya_ring','nenya_ring'))):
            jinfo=self.joints[j]
            Rj=R@rot_rpy(jinfo['origin_rpy'])
            jp=p+R@np.asarray(jinfo['origin_mm'],float)/1000.
            axis=Rj@np.asarray(jinfo['axis'],float)
            axis/=np.linalg.norm(axis)
            pivots.append(jp);axes.append(axis)
            R=Rj@rot_axis(jinfo['axis'],q[idx])
            p=jp
            poses[child]=(R,p)
        j=self.joints['gyro_to_frame']
        R=R@rot_rpy(j['origin_rpy'])
        p=p+poses['nenya_ring'][0]@(np.asarray(j['origin_mm'],float)/1000.)
        poses['drone_body_link']=(R,p)
        for idx in range(1,5):
            key='motor_'+str(idx)
            motor=self.joints[key]
            Rp=R@rot_rpy(motor['origin_rpy'])
            pp=p+R@np.asarray(motor['origin_mm'],float)/1000.
            poses[f'prop_{idx}_link']=(Rp,pp)
        return poses,np.array(pivots),np.array(axes)

    def jacobian(self,name,poses,pivots,axes,at_com=True):
        order={'narya_ring':1,'vilya_ring':2,'nenya_ring':3,
               'drone_body_link':3,'prop_1_link':3,'prop_2_link':3,
               'prop_3_link':3,'prop_4_link':3}
        R,p=poses[name]
        point=p + (R@np.asarray(self.links[name]['com_mm'],float)/1000.
                   if at_com else np.zeros(3))
        Jv=np.zeros((3,3));Jw=np.zeros((3,3))
        for i in range(order[name]):
            Jw[:,i]=axes[i]
            Jv[:,i]=np.cross(axes[i],point-pivots[i])
        return Jv,Jw

    def mass_gravity(self,q):
        poses,pivots,axes=self.kinematics(q)
        M=np.zeros((3,3));g=np.zeros(3)
        for name in MOVING:
            R,_=poses[name]
            Jv,Jw=self.jacobian(name,poses,pivots,axes)
            mass=self.masses[name]
            M+=mass*(Jv.T@Jv)+Jw.T@R@self.inertia[name]@R.T@Jw
            g+=mass*self.gravity*Jv[2,:]
        return 0.5*(M+M.T),g

    def potential(self,q):
        poses,_,_=self.kinematics(q)
        return sum(self.masses[name]*self.gravity*(p+R@np.array(self.links[name]['com_mm'],float)/1000.)[2]
                   for name,(R,p) in ((name,poses[name]) for name in MOVING))

    def coriolis(self,q,qd,eps=1.e-4):
        q=np.asarray(q,float);qd=np.asarray(qd,float)
        dM=np.empty((3,3,3))
        for k in range(3):
            dq=np.zeros(3);dq[k]=eps
            dM[:,:,k]=(self.mass_gravity(q+dq)[0]-self.mass_gravity(q-dq)[0])/(2.*eps)
        # c_i = sum_jk [dM_ij/dqk - 0.5*dM_jk/dqi] qdot_j qdot_k
        return np.einsum('ijk,j,k->i',dM,qd,qd)-0.5*np.einsum('jki,j,k->i',dM,qd,qd)

    def rotation_error_and_rates(self,q,R_world_body,body_rates):
        poses,pivots,axes=self.kinematics(q)
        R=poses['drone_body_link'][0]
        Jr=R.T@axes.T
        rate=np.linalg.lstsq(Jr,np.asarray(body_rates,float),rcond=None)[0]
        err=Rotation.from_matrix(np.asarray(R_world_body)@R.T).as_rotvec()
        return err,Jr,rate

    def estimate_joints(self,R_meas,q_guess,iterations=12):
        """Damped orientation IK from the IMU quaternion; no external joint sensor.

        Returns estimated q, residual rad, condition number. A measurement at
        pitch singularity or with a large residual must not be used for control.
        """
        q=np.array(q_guess,float).copy()
        for _ in range(iterations):
            poses,pivots,axes=self.kinematics(q)
            pred=poses['drone_body_link'][0]
            err=Rotation.from_matrix(R_meas@pred.T).as_rotvec()
            if np.linalg.norm(err)<1e-7:break
            J=axes.T
            delta=np.linalg.solve(J.T@J+1e-4*np.eye(3),J.T@err)
            delta=np.clip(delta,-0.45,0.45)
            q+=delta
        poses,pivots,axes=self.kinematics(q)
        residual=np.linalg.norm(Rotation.from_matrix(R_meas@poses['drone_body_link'][0].T).as_rotvec())
        cond=np.linalg.cond(axes.T)
        return q,float(residual),float(cond)

    def motor_effectiveness(self,q,kf,km):
        """Map RPM^2 to joint generalized torques for rotor_system.cpp.

        Wrenches use motor positions wrt drone_body_link origin, then project
        through spatial Jacobians; this includes the effects of collective thrust
        and lever arms on the tilted gimbal.
        """
        poses,pivots,axes=self.kinematics(q)
        R,p=poses['drone_body_link']
        Jv,Jw=self.jacobian('drone_body_link',poses,pivots,axes,at_com=False)
        out=np.zeros((3,4))
        for i,(x,y,z) in enumerate(self.motor_positions):
            force=np.array([0.,0.,kf])
            torque=np.array([y*kf,-x*kf,self.motor_spin[i]*km])
            out[:,i]=Jv.T@(R@force)+Jw.T@(R@torque)
        return out


def fifth_order(t, duration):
    z = min(1., max(0., t / duration))
    s = 10*z**3-15*z**4+6*z**5
    sd = (30*z*z-60*z**3+30*z**4) / duration
    sdd = (60*z-180*z*z+120*z**3) / (duration**2)
    return s, sd, sdd


def lqr_gains(M, damping, angle_scale, rate_scale, torque_scale):
    # Gravity is cancelled in feedback linearization; use torque increments.
    Minv=np.linalg.inv(M)
    A=np.block([[np.zeros((3,3)),np.eye(3)],
                [np.zeros((3,3)),-Minv@np.diag(damping)]])
    B=np.vstack([np.zeros((3,3)),Minv])
    weights=np.radians(np.array(list(angle_scale)+list(rate_scale),float))
    Q=np.diag(1./weights**2)
    R=np.diag(1./np.asarray(torque_scale,float)**2)
    S=solve_continuous_are(A,B,Q,R)
    K=np.linalg.solve(R,B.T@S)
    if np.max(np.real(np.linalg.eigvals(A-B@K)))>=-1e-5:
        raise ValueError('LQR not stabilizing')
    return K


def allocate_motors(G,target_q_nm,nominal_rpm,max_rpm,reg=0.04,axis_weights=(1.,1.,1.)):
    """Bounded least-squares allocation using actual G(q), no fake 4x4 mixer.

    G maps rotor RPM^2 to generalized yaw/pitch/roll torques. A fourth
    regularization objective keeps the motors near a feasible nominal speed.
    If requested generalized torque is infeasible, residual reports it.
    """
    max_rpm=float(max_rpm)
    H=G*(max_rpm**2)
    u0=np.ones(4)*(float(nominal_rpm)/max_rpm)**2
    weights=np.asarray(axis_weights,float)
    if weights.shape!=(3,) or not np.all(np.isfinite(weights)) or np.any(weights<=0):
        raise ValueError('axis_weights must have three finite positive elements')
    A=np.vstack([weights[:,None]*H,reg*np.eye(4)])
    b=np.concatenate([weights*np.asarray(target_q_nm,float),reg*u0])
    # Solve bounded RPM^2, avoid artificial negative thrust.
    fit=lsq_linear(A,b,bounds=(np.zeros(4),np.ones(4)),tol=1e-7,max_iter=40)
    x=np.clip(fit.x,0.,1.)
    return max_rpm*np.sqrt(x), float(np.linalg.norm(H@x-target_q_nm)),H@x
