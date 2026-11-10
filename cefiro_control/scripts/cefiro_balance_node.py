#!/usr/bin/env python3
"""CEFIRO SIM-only 3-DOF URDF-based swing-up + local LQR.

Uses actual yaw/pitch/roll joint axes and all moving-link inertias in
cefiro_description/config/cefiro_params.yaml. The Gazebo IMU provides a
world-frame quaternion; joint coordinates are estimated by kinematic inversion.
No direct hardware actuation is supported or validated. Never operate alongside
another RPM publisher. Physical parameters are still CAD approximations.
"""
import math
import json
import time
from pathlib import Path

import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField, FluidPressure
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, String, Float64MultiArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from cefiro_msgs.msg import MotorRpm, MotorTelemetry
from cefiro_multibody import CefiroMultibody, wrap_delta, fifth_order, lqr_gains, allocate_motors
from reference_envelope import estimate_reference_envelope




class BalanceNode(Node):
    ACTUATING={'ARMING','SWINGUP_ARMING','SPINUP','SWINGUP','BRAKING','CATCHING','YAW_ALIGN','CAPTURED'}

    def __init__(self):
        super().__init__('cefiro_balance')
        # use_sim_time is declared by rclpy/launch in Jazzy; do not redeclare.
        declare=self.declare_parameter
        declare('simulation_only',True)
        declare('enable_swingup',True)
        declare('model_yaml','')
        declare('spawn_rpy_rad',[0.,0.,1.07])
        declare('update_rate_hz',50.)
        declare('swingup_duration_s',11.)
        declare('swingup_timeout_s',36.)
        declare('braking_timeout_s',12.)
        declare('spinup_hold_s',0.9)
        declare('spinup_timeout_s',7.)
        declare('spinup_fraction',0.40)
        # Torque is in URDF joint coordinates: [yaw,pitch,roll].
        declare('trajectory_kp_nm_rad',[0.12,0.65,0.9])
        declare('trajectory_kd_nm_s_rad',[0.14,0.32,0.32])
        declare('max_generalized_torque_nm',[0.40,1.25,1.25])
        declare('lqr_angle_scale_deg',[14.,12.,12.])
        declare('lqr_rate_scale_deg_s',[70.,65.,65.])
        declare('lqr_torque_scale_nm',[0.07,0.20,0.20])
        declare('yaw_return_duration_s',20.)
        declare('lqr_blend_duration_s',1.8)
        declare('catch_min_hold_s',2.5)
        declare('catch_timeout_s',18.)
        declare('yaw_align_timeout_s',35.)
        declare('brake_angle_deg',12.)
        declare('brake_rate_deg_s',18.)
        declare('brake_hold_s',0.35)
        declare('yaw_start_angle_deg',6.)
        declare('yaw_start_rate_deg_s',14.)
        declare('yaw_motion_angle_deg',9.)
        declare('yaw_motion_rate_deg_s',25.)
        declare('catch_yaw_torque_nm',0.075)
        declare('yaw_align_torque_nm',0.10)
        declare('local_escape_angle_deg',35.)
        declare('local_escape_rate_deg_s',100.)
        declare('local_escape_hold_s',0.35)
        declare('joint_rate_filter_alpha',0.45)
        declare('catch_angle_deg',32.)
        declare('catch_rate_deg_s',90.)
        declare('capture_roll_pitch_deg',9.)
        declare('capture_yaw_deg',14.)
        declare('capture_rate_deg_s',25.)
        declare('capture_hold_s',0.55)
        declare('max_angle_deg',170.)
        declare('max_rate_deg_s',400.)
        declare('max_joint_ik_condition',15.)
        declare('max_joint_ik_residual_deg',2.)
        declare('nominal_rpm',4700.)
        declare('max_rpm',5400.)
        declare('rpm_slew_per_s',10000.)
        declare('max_allocation_residual_nm',0.22)
        declare('allocation_fault_hold_s',1.0)
        declare('max_rpm_tracking_error',650.0)
        declare('rpm_tracking_fault_hold_s',1.2)
        declare('imu_timeout_s',0.25)
        declare('mag_timeout_s',0.60)
        declare('actuator_timeout_s',0.45)
        declare('reference_timeout_s',0.9)
        # Requested design-space ceilings [BODY roll, pitch, yaw]. The actual
        # limits are REDUCED at startup by CAD kinematics + static motor authority.
        # Yaw +/-180 covers 360 degrees of headings, not a full-turn trajectory.
        # No physical stability claim until reference steps are validated in Gazebo.
        declare('reference_limits_deg',[30.0,30.0,180.0])
        declare('reference_joint_limits_deg',[180.5,30.0,30.0])
        declare('reference_slew_deg_s',[6.0,6.0,8.0])
        declare('reference_accel_deg_s2',[18.0,18.0,24.0])
        # SIM-only residual-trim loop; enabled ONLY in CAPTURED.
        # Gains are torque per integrated radian error (Nm/(rad*s)).
        declare('reference_integral_gain_nm_rad_s',[0.12,0.45,0.45])
        declare('reference_integral_limit_nm',[0.055,0.12,0.12])
        declare('reference_integral_leak_per_s',0.015)
        declare('reference_integral_allocation_gate_nm',0.025)
        declare('reference_integral_rpm_gate',450.0)
        declare('reference_integral_target_rate_gate_deg_s',0.8)

        declare('arm_ack_timeout_s',0.75)
        declare('require_magnetometer',True)
        declare('prop_ct',0.10)
        declare('prop_cq',0.015)
        declare('prop_diameter_m',0.254)
        declare('air_density_kg_m3',1.225)
        self.param=lambda key:self.get_parameter(key).value
        self.permitted=bool(self.param('simulation_only')) and bool(self.param('use_sim_time'))
        if not self.permitted:
            self.get_logger().error('HARD LOCKOUT: simulation_only and use_sim_time must be true.')
        model_file=str(self.param('model_yaml'))
        if not model_file:
            from ament_index_python.packages import get_package_share_directory
            model_file=str(Path(get_package_share_directory('cefiro_description'))/'config'/'cefiro_params.yaml')
        self.model=CefiroMultibody(model_file,spawn_rpy=self.param('spawn_rpy_rad'))
        self.M0,self.g0=self.model.mass_gravity([0.,0.,0.])
        self.damping=np.array([self.model.joints[x]['damping'] for x in ('yaw','pitch','roll')],float)
        self.K=lqr_gains(self.M0,self.damping,self.param('lqr_angle_scale_deg'),
                         self.param('lqr_rate_scale_deg_s'),self.param('lqr_torque_scale_nm'))
        rho=float(self.param('air_density_kg_m3'))
        D=float(self.param('prop_diameter_m'))
        self.kf=float(self.param('prop_ct'))*rho*D**4/3600.
        self.km=float(self.param('prop_cq'))*rho*D**5/3600.
        # Evaluate achievable STATIC attitude targets using the same URDF CAD
        # kinematics, gravity feedforward and bounded RPM^2 allocation as the
        # active controller. Do not claim global nonlinear stability from this.
        nominal_rotation=self.model.kinematics(np.zeros(3))[0]['drone_body_link'][0]
        try:
            self.reference_envelope_deg, envelope_report = estimate_reference_envelope(
                self.model, nominal_rotation,
                self.param('reference_limits_deg'),
                self.param('reference_joint_limits_deg'),
                self.param('nominal_rpm'), self.param('max_rpm'),
                [min(self.param('max_generalized_torque_nm')[0],
                     self.param('yaw_align_torque_nm'))] +
                list(self.param('max_generalized_torque_nm')[1:]),
                self.param('max_allocation_residual_nm'), self.kf, self.km)
            if not envelope_report.get('valid'):
                self.get_logger().error('STATIC ENVELOPE INVALID: '+str(envelope_report))
        except Exception as error:  # Never leave the GUI assuming unsupported limits.
            self.reference_envelope_deg=np.array([5.,5.,10.])
            self.get_logger().error('Static envelope analysis failed; using pilot limits: '+str(error))
        self.get_logger().warn(
            'SIM-only MODEL ESTIMATE (NOT stability tested): R=+/-%.1f, P=+/-%.1f, Y=+/-%.1f deg. '
            'Yaw selects heading by shortest path; test in Gazebo before relying on large angles.'
            % tuple(self.reference_envelope_deg))
        self.enabled=False;self.estop=False;self.fault=None;self.state='DISABLED'
        self.mode='';self.strategy='';self.active_strategy=None
        # References sent by the GUI are body-frame RPY deviations from the
        # nominal horizontal attitude (NOT the gimbal's joint angles).
        self.reference=np.zeros(3);self.last_reference=None
        self.nominal_body_rotation=self.model.kinematics(np.zeros(3))[0]['drone_body_link'][0]
        self.filtered_q_ref=np.zeros(3)
        self.filtered_q_ref_velocity=np.zeros(3)
        self.filtered_q_ref_acceleration=np.zeros(3)
        self.reference_integral_torque=np.zeros(3)
        self.last_allocation_residual=0.0
        self.last_requested_q_ref=np.zeros(3)

        self.last_imu=None;self.last_mag=None;self.last_actuator=None
        self.R_meas=None;self.body_rates=None;self.armed=False;self.actuator_watchdog_ok=False;self.measured_rpm=np.zeros(4)
        self.q_est=None;self.qd_est=None;self.ik_residual=None;self.ik_condition=None
        self.command=np.zeros(4)
        self.arm_time=None;self.spinup_time=None;self.swing_time=None
        self.swing_from=None;self.swing_to=None;self.cross_hold=None
        self.yaw_hold=None;self.yaw_catch_start=None;self.catch_time=None
        self.brake_time=None;self.brake_hold=None;self.blend_time=None
        self.yaw_align_time=None;self.yaw_elapsed=0.;self.yaw_start=None;self.yaw_end=None
        self.yaw_stable_since=None;self.local_bad_since=None;self.last_diag=0.
        self.last_tau=np.zeros(3);self.last_achieved=np.zeros(3);self.last_target=np.zeros(3)
        self.allocation_bad_since=None;self.rpm_bad_since=None;self.last_log=0.;self.last_status=0.
        self.last_tick=time.monotonic()
        self.motor_pub=self.create_publisher(MotorRpm,'/cefiro_actuators/target_rpm',10)
        self.arm_pub=self.create_publisher(Bool,'/cefiro_actuators/armed',10)
        self.status_pub=self.create_publisher(String,'/cefiro/control/status',10)
        self.diag_pub=self.create_publisher(String,'/cefiro/control/diagnostics',10)
        reference_qos=QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reference_envelope_pub=self.create_publisher(
            Float64MultiArray,'/cefiro/control/reference_envelope_deg',reference_qos)
        self.create_timer(1.0,self.publish_reference_envelope)
        self.publish_reference_envelope()
        self.create_subscription(Imu,'/imu/data',self.on_imu,qos_profile_sensor_data)
        self.create_subscription(MagneticField,'/magnetometer/data',self.on_mag,qos_profile_sensor_data)
        self.create_subscription(FluidPressure,'/bmp581/air_pressure',lambda m:None,qos_profile_sensor_data)
        self.create_subscription(MotorTelemetry,'/cefiro_actuators/telemetry',self.on_telemetry,10)
        self.create_subscription(Vector3Stamped,'/cefiro/control/attitude_reference',self.on_ref,10)
        self.create_subscription(Bool,'/cefiro/control/enable',self.on_enable,10)
        self.create_subscription(Bool,'/cefiro/control/emergency_stop',self.on_stop,10)
        self.create_subscription(String,'/cefiro/control/mode',lambda m:setattr(self,'mode',m.data),10)
        self.create_subscription(String,'/cefiro/control/strategy',lambda m:setattr(self,'strategy',m.data),10)
        self.create_timer(1./float(self.param('update_rate_hz')),self.update)
        self.get_logger().warn('CEFIRO URDF 3DOF SWING-UP SIM ONLY; joint order YAW/PITCH/ROLL; '
                               f'Jdiag={np.diag(self.M0).round(5)}, g0={self.g0.round(5)} Nm')

    def on_imu(self,msg):
        a=np.array([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w],float)
        rates=np.array([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z],float)
        if (msg.orientation_covariance[0]==-1 or not np.all(np.isfinite(a)) or
            np.linalg.norm(a)<0.1 or not np.all(np.isfinite(rates))):
            self.R_meas=None
            return
        self.R_meas=Rotation.from_quat(a/np.linalg.norm(a)).as_matrix()
        self.body_rates=rates
        self.last_imu=time.monotonic()

    def on_mag(self,msg):
        a=np.array([msg.magnetic_field.x,msg.magnetic_field.y,msg.magnetic_field.z],float)
        # Gazebo currently emits an unrealistically large field. Freshness only:
        # heading comes from the simulation IMU quaternion, not this magnetometer.
        if np.all(np.isfinite(a)) and np.linalg.norm(a)>1.e-9:
            self.last_mag=time.monotonic()

    def on_telemetry(self,msg):
        self.last_actuator=time.monotonic()
        self.armed=bool(msg.armed)
        self.actuator_watchdog_ok=bool(msg.watchdog_ok)
        rpms=np.array(msg.measured_rpm,float)
        if rpms.shape==(4,) and np.all(np.isfinite(rpms)):
            self.measured_rpm=rpms

    def publish_reference_envelope(self):
        msg=Float64MultiArray()
        msg.data=[float(x) for x in self.reference_envelope_deg]
        self.reference_envelope_pub.publish(msg)

    def on_ref(self,msg):
        a=np.array([msg.vector.x,msg.vector.y,msg.vector.z],float)
        if not np.all(np.isfinite(a)):
            return
        # Defend against alternate ROS publishers bypassing the GUI limits.
        limits=np.radians(self.reference_envelope_deg)
        if limits.shape!=(3,) or np.any(limits<=0.):
            self.abort('INVALID_REFERENCE_LIMITS_CONFIG')
            return
        self.reference=np.clip(a,-limits,limits)
        # The GUI must remain a continuously publishing command heartbeat.
        self.last_reference=time.monotonic()

    def update_filtered_reference(self,dt):
        """Map bounded BODY Euler offsets to URDF joint targets, slew them.

        q ordering is [yaw,pitch,roll]. Small body roll mainly actuates the
        *pitch* joint in this URDF, and body pitch mainly actuates *roll*.
        Do not re-use body Euler angles directly as joint coordinates.
        """
        desired_body_offset=Rotation.from_euler('xyz',self.reference).as_matrix()
        world_desired=self.nominal_body_rotation@desired_body_offset
        qgoal,res,cond=self.model.estimate_joints(world_desired,self.filtered_q_ref)
        if not np.all(np.isfinite(qgoal)) or res>math.radians(0.3) or cond>10.:
            self.abort('REFERENCE_IK_INVALID')
            return None
        # Select the shortest angular branch; yaw is a continuous heading
        # over +/-180 deg. The two horizontal joints retain a conservative
        # margin below the 35 deg local-escape threshold.
        qgoal=wrap_delta(qgoal)
        joint_limits=np.asarray(self.param('reference_joint_limits_deg'),float)
        if joint_limits.shape!=(3,) or not np.all(np.isfinite(joint_limits)):
            self.abort('INVALID_REFERENCE_JOINT_LIMITS')
            return None
        if np.any(np.abs(np.degrees(qgoal))>joint_limits+1.e-3):
            self.abort('REFERENCE_JOINT_ENVELOPE')
            return None
        self.last_requested_q_ref=qgoal.copy()
        # GUI limits [body roll,pitch,yaw] -> joint limits [yaw,pitch,roll].
        ordered=[2,0,1]
        vmax=np.radians(np.asarray(self.param('reference_slew_deg_s'),float)[ordered])
        amax=np.radians(np.asarray(self.param('reference_accel_deg_s2'),float)[ordered])
        error=wrap_delta(qgoal-self.filtered_q_ref)
        # Continuously accelerate/decelerate the internal target, and brake
        # early enough to avoid overshooting a larger heading/tilt request.
        desired_speed=np.sign(error)*np.minimum(vmax,np.sqrt(2.*amax*np.abs(error)))
        previous_velocity=self.filtered_q_ref_velocity.copy()
        dv=np.clip(desired_speed-self.filtered_q_ref_velocity,-amax*dt,amax*dt)
        self.filtered_q_ref_velocity=np.clip(self.filtered_q_ref_velocity+dv,-vmax,vmax)
        delta=self.filtered_q_ref_velocity*dt
        reached=(np.abs(delta)>=np.abs(error)) & (np.sign(delta)==np.sign(error))
        self.filtered_q_ref+=np.where(reached,error,delta)
        self.filtered_q_ref_velocity[reached]=0.
        # Bounded reference acceleration for model-based feedforward.
        self.filtered_q_ref_acceleration=np.clip(
            (self.filtered_q_ref_velocity-previous_velocity)/max(dt,1.e-4),
            -amax,amax)
        self.filtered_q_ref_acceleration[reached]=0.
        self.filtered_q_ref=wrap_delta(self.filtered_q_ref)
        return self.filtered_q_ref.copy()

    def on_enable(self,msg):
        if not msg.data:
            self.enabled=False;self.estop=False;self.fault=None
            self.stop('DISABLED')
        elif not self.enabled:
            self.enabled=True
            self.fault=None

    def on_stop(self,msg):
        if msg.data:
            self.estop=True
            self.abort('EMERGENCY_STOP')

    def status(self,label,force=False):
        now=time.monotonic()
        if label!=self.state or force or now-self.last_status>0.5:
            self.state=label;self.last_status=now
            self.status_pub.publish(String(data='BALANCE: '+label))

    def stop(self,label):
        self.arm_time=None;self.spinup_time=None;self.swing_time=None
        self.swing_from=None;self.swing_to=None;self.cross_hold=None
        self.catch_time=None;self.yaw_hold=None;self.yaw_catch_start=None
        self.brake_time=None;self.brake_hold=None;self.blend_time=None
        self.yaw_align_time=None;self.yaw_elapsed=0.;self.yaw_start=None;self.yaw_end=None
        self.yaw_stable_since=None;self.local_bad_since=None
        self.allocation_bad_since=None;self.rpm_bad_since=None;self.active_strategy=None
        self.command[:]=0.
        # Clear stale attitude targets on stop/fault; no automatic re-apply
        # after a new capture unless the operator sends another reference.
        self.reference[:]=0.
        self.filtered_q_ref[:]=0.
        self.filtered_q_ref_velocity[:]=0.
        self.filtered_q_ref_acceleration[:]=0.
        self.reference_integral_torque[:]=0.
        self.last_allocation_residual=0.0
        self.last_requested_q_ref[:]=0.
        self.motor_pub.publish(MotorRpm(data=[0.]*4))
        self.arm_pub.publish(Bool(data=False))
        self.status(label,True)

    def abort(self,why):
        self.fault=why
        self.stop('FAULT_LATCHED: '+why)
        self.get_logger().error('CEFIRO: '+why)

    def estimate_state(self):
        # Warm-start IK from last joint angles. At startup check both hanging
        # branches and choose the branch with small |q_pitch|, then |q_roll|.
        guesses=[self.q_est] if self.q_est is not None else [
            [0.,0.,0.],[0.,0.,math.pi],[0.,0.,-math.pi],[0.,math.pi,0.]]
        solutions=[self.model.estimate_joints(self.R_meas,x) for x in guesses]
        if self.q_est is None:
            candidates=[(np.linalg.norm(wrap_delta(q[1:])),q,res,cnd)
                        for q,res,cnd in solutions if res<0.04]
            if candidates:
                # A near-hanging state is expected to have |pitch| small and |roll|~pi.
                _,q,res,cnd=min(candidates,key=lambda a:abs(wrap_delta(a[1][1])))
            else:
                q,res,cnd=min(solutions,key=lambda a:a[1])
        else:
            q,res,cnd=solutions[0]
        self.q_est=np.asarray(q,float)
        self.ik_residual=res;self.ik_condition=cnd
        poses,_,axes=self.model.kinematics(q)
        R=poses['drone_body_link'][0]
        body_J=R.T@axes.T
        qd_raw=np.linalg.lstsq(body_J,self.body_rates,rcond=None)[0]
        # Small low-pass filter on measured body rates; no Euler differentiation.
        alpha=float(np.clip(self.param('joint_rate_filter_alpha'),0.05,1.0))
        if self.qd_est is None:
            self.qd_est=qd_raw
        else:
            self.qd_est=(1.0-alpha)*self.qd_est+alpha*qd_raw
        return (res<=math.radians(self.param('max_joint_ik_residual_deg'))
                and cnd<=self.param('max_joint_ik_condition')
                and np.all(np.isfinite(self.qd_est)))

    def send_command(self,tau,dt,now,phase):
        tau=np.asarray(tau,float)
        if not np.all(np.isfinite(tau)):
            self.abort('NONFINITE_COMMAND');return False
        # Saturation is on generalized joint torques [yaw,pitch,roll], not on body axes.
        tau=np.clip(tau,-np.array(self.param('max_generalized_torque_nm')),
                    np.array(self.param('max_generalized_torque_nm')))
        G=self.model.motor_effectiveness(self.q_est,self.kf,self.km)
        # Pitch/roll authority is prioritized over yaw during the capture.
        weights=(0.35,1.0,1.0) if phase in ('BRAKING','CATCHING','YAW_ALIGN','CAPTURED') else (0.7,1.0,1.0)
        rpms,residual,achieved=allocate_motors(
            G,tau,self.param('nominal_rpm'),self.param('max_rpm'),axis_weights=weights)
        self.last_tau=tau.copy();self.last_achieved=achieved.copy()
        self.last_allocation_residual=float(residual)
        if not np.all(np.isfinite(rpms)):
            self.abort('ALLOCATOR_NONFINITE');return False
        if residual>self.param('max_allocation_residual_nm'):
            if self.allocation_bad_since is None:self.allocation_bad_since=now
            if now-self.allocation_bad_since>self.param('allocation_fault_hold_s'):
                self.abort(f'ALLOCATION_INFEASIBLE_{residual:.2f}Nm');return False
        else:self.allocation_bad_since=None
        step=float(self.param('rpm_slew_per_s'))*dt
        self.command=np.clip(rpms,self.command-step,self.command+step)
        # Detect sustained mismatch between requested and actual RPM. The ESC
        # model is voltage-limited and cannot always deliver its setpoint.
        if (self.spinup_time is not None and now-self.spinup_time>2.0 and
            np.max(np.abs(self.command-self.measured_rpm))>self.param('max_rpm_tracking_error')):
            if self.rpm_bad_since is None:self.rpm_bad_since=now
            if now-self.rpm_bad_since>self.param('rpm_tracking_fault_hold_s'):
                self.abort('RPM_TRACKING_SATURATION');return False
        else:self.rpm_bad_since=None
        self.motor_pub.publish(MotorRpm(data=self.command.tolist()))
        self.status(phase)
        if now-self.last_diag>=0.1:
            self.last_diag=now
            diagnostic={
                'phase':phase,
                'q_deg':np.degrees(wrap_delta(self.q_est)).round(2).tolist(),
                'qdot_deg_s':np.degrees(self.qd_est).round(2).tolist(),
                'target_deg':np.degrees(wrap_delta(self.last_target)).round(2).tolist(),
                'torque_requested_nm':tau.round(4).tolist(),
                'torque_allocated_nm':achieved.round(4).tolist(),
                'torque_measured_est_nm':(G@(self.measured_rpm**2)).round(4).tolist(),
                'allocation_residual_nm':round(float(residual),4),
                'command_rpm':self.command.round(0).tolist(),
                'measured_rpm':self.measured_rpm.round(0).tolist(),
                'joint_ik_condition':round(float(self.ik_condition),2),
                'gui_body_reference_deg':np.degrees(self.reference).round(2).tolist(),
                'filtered_joint_reference_deg':np.degrees(self.filtered_q_ref).round(2).tolist(),
                'reference_enabled':phase=='CAPTURED',
                'reference_integral_torque_nm':self.reference_integral_torque.round(4).tolist(),
                'filtered_joint_reference_rate_deg_s':np.degrees(self.filtered_q_ref_velocity).round(2).tolist(),
                'joint_tracking_error_deg':np.degrees(wrap_delta(self.last_target-self.q_est)).round(2).tolist(),
            }
            self.diag_pub.publish(String(data=json.dumps(diagnostic,separators=(',',':'))))
        if now-self.last_log>2.0:
            self.last_log=now
            self.get_logger().info(f'{phase} q[Y,P,R]={np.degrees(wrap_delta(self.q_est)).round(1)} deg '
                f'qd={np.degrees(self.qd_est).round(1)} deg/s '
                f'Qcmd={tau.round(2)} Nm residual={residual:.3f} Nm '
                f'Qmeas={(G@(self.measured_rpm**2)).round(2)} Nm '
                f'RPMcmd={self.command.round(0)} RPMmeas={self.measured_rpm.round(0)}')
        return True

    def controller(self,q_target,qd_target,qdd_target,use_lqr=False):
        q=self.q_est;qd=self.qd_est
        error=wrap_delta(q-q_target)
        # Preserve unwrapped roll error on the planned branch.
        if not use_lqr:
            error[2]=q[2]-q_target[2]
        rate_error=qd-qd_target
        M,g=self.model.mass_gravity(q)
        c=self.model.coriolis(q,qd)
        if use_lqr:
            feedback=-self.K@np.concatenate([error,rate_error])
        else:
            feedback=-(np.array(self.param('trajectory_kp_nm_rad'))*error+
                       np.array(self.param('trajectory_kd_nm_s_rad'))*rate_error)
        return M@qdd_target+c+g+self.damping*qd+feedback

    def captured_integral_correction(self, q_target, dt):
        """Limited error trim ONLY in CAPTURED, with anti-windup gates.

        Integral is stored as a *torque*, so it can be directly saturated in Nm.
        During moving references, actuator mismatch, or infeasible allocation
        the integrator holds/leaks; it must not demand unattainable torque.
        """
        gains=np.asarray(self.param('reference_integral_gain_nm_rad_s'),float)
        limits=np.asarray(self.param('reference_integral_limit_nm'),float)
        if gains.shape!=(3,) or limits.shape!=(3,) or np.any(gains<0) or np.any(limits<0):
            self.abort('INVALID_INTEGRAL_CONFIG')
            return np.zeros(3)
        leak=max(0.,float(self.param('reference_integral_leak_per_s')))
        self.reference_integral_torque*=max(0.,1.-leak*dt)
        error=wrap_delta(q_target-self.q_est)
        target_rate_gate=math.radians(float(self.param('reference_integral_target_rate_gate_deg_s')))
        ready=(self.last_allocation_residual < float(self.param('reference_integral_allocation_gate_nm'))
               and np.max(np.abs(self.command-self.measured_rpm))
               < float(self.param('reference_integral_rpm_gate')))
        if ready:
            axes=np.abs(self.filtered_q_ref_velocity)<target_rate_gate
            self.reference_integral_torque+=np.where(axes,gains*error*dt,0.)
        self.reference_integral_torque=np.clip(self.reference_integral_torque,-limits,limits)
        return self.reference_integral_torque.copy()

    def blended_controller(self, q_target, qd_target, qdd_target, blend):
        # Both laws use identical reference, gravity and inertia feedforward.
        # Only their feedback terms differ, giving a bumpless handoff.
        pure_pd=self.controller(q_target,qd_target,qdd_target,use_lqr=False)
        pure_lqr=self.controller(q_target,qd_target,qdd_target,use_lqr=True)
        return (1.0-blend)*pure_pd+blend*pure_lqr

    def local_condition(self,angle_deg,rate_deg_s):
        # Only horizontal joints; yaw is intentionally handled later.
        return (np.max(np.abs(np.degrees(wrap_delta(self.q_est[1:]))))<angle_deg
            and np.max(np.abs(np.degrees(self.qd_est[1:])))<rate_deg_s)

    def local_envelope_ok(self,now):
        if self.local_condition(self.param('local_escape_angle_deg'),
                                self.param('local_escape_rate_deg_s')):
            self.local_bad_since=None
            return True
        if self.local_bad_since is None:self.local_bad_since=now
        return (now-self.local_bad_since) <= self.param('local_escape_hold_s')

    def update(self):
        now=time.monotonic()
        dt=max(0.001,min(.05,now-self.last_tick));self.last_tick=now
        if not self.permitted:
            self.status('SIM_ONLY_LOCKOUT');return
        if not self.enabled or self.estop or self.fault:
            self.status('FAULT_LATCHED: '+self.fault if self.fault else 'DISABLED');return
        swing=self.strategy=='SWING-UP + LQR (SIM)'
        lqr=self.strategy=='LQR'
        if self.mode!='ATTITUDE' or not (swing or lqr):
            if self.state in self.ACTUATING:self.abort('GUI_MODE_CHANGED')
            else:self.status('WAIT_GUI_AUTOMATIC_LQR')
            return
        if self.state in self.ACTUATING and self.strategy!=self.active_strategy:
            self.abort('STRATEGY_CHANGED');return
        if swing and not self.param('enable_swingup'):
            self.status('SWINGUP_DISABLED_IN_CONFIG');return
        prerequisites=[
            (self.last_imu is not None and self.R_meas is not None and
             now-self.last_imu<self.param('imu_timeout_s'),'IMU'),
            (not self.param('require_magnetometer') or
             (self.last_mag is not None and now-self.last_mag<self.param('mag_timeout_s')),'MAG'),
            (self.last_reference is not None and
             now-self.last_reference<self.param('reference_timeout_s'),'REFERENCE'),
            # During swing-up / capture the GUI must send equilibrium 0,0,0;
            # small nonzero pilot references are only allowed in CAPTURED.
            (self.state=='CAPTURED' or
             np.linalg.norm(self.reference)<math.radians(.5),'NONZERO_REFERENCE'),
            (self.last_actuator is not None and
             now-self.last_actuator<self.param('actuator_timeout_s'),'ACTUATOR')]
        for valid,reason in prerequisites:
            if not valid:
                if self.state in self.ACTUATING:self.abort('LOST_'+reason)
                else:self.status('WAIT_'+reason)
                return
        if not self.estimate_state():
            if self.state in self.ACTUATING:self.abort('JOINT_IK_SINGULAR_OR_INVALID')
            else:self.status('WAIT_JOINT_ESTIMATE')
            return
        if np.max(np.abs(np.degrees(self.qd_est)))>self.param('max_rate_deg_s'):
            self.abort('JOINT_RATE_LIMIT');return
        # No simultaneous GUI manual motor publishing is permitted.
        if self.state in ('SWINGUP_ARMING','ARMING'):
            if not self.armed:
                if now-self.arm_time>self.param('arm_ack_timeout_s'):
                    self.abort('ARM_ACK_TIMEOUT')
                else:self.status(self.state)
                return
            self.spinup_time=now
            self.status('SPINUP',True)
            return
        if self.state in ('SPINUP','SWINGUP','BRAKING','CATCHING','YAW_ALIGN','CAPTURED'):
            if not self.armed:
                self.abort('UNEXPECTED_DISARM');return

            if self.state=='SPINUP':
                if now-self.spinup_time>self.param('spinup_timeout_s'):
                    self.abort('SPINUP_TIMEOUT');return
                target=(self.model.mass_gravity(self.q_est)[1]
                        if self.active_strategy=='SWING-UP + LQR (SIM)'
                        else self.controller(np.zeros(3),np.zeros(3),np.zeros(3),use_lqr=True))
                if not self.send_command(target,dt,now,'SPINUP'):return
                if (now-self.spinup_time>=self.param('spinup_hold_s') and
                    np.mean(self.measured_rpm)>self.param('spinup_fraction')*self.param('nominal_rpm')):
                    if self.active_strategy=='LQR':
                        if (not self.local_condition(self.param('capture_roll_pitch_deg'),
                                                     self.param('capture_rate_deg_s')) or
                            abs(math.degrees(wrap_delta(self.q_est[0]))) >= self.param('capture_yaw_deg') or
                            abs(math.degrees(self.qd_est[0])) >= self.param('capture_rate_deg_s')):
                            self.abort('LEFT_MANUAL_CAPTURE_DURING_SPINUP');return
                        self.filtered_q_ref[:]=0.
                        self.filtered_q_ref_velocity[:]=0.
                        self.filtered_q_ref_acceleration[:]=0.
                        self.reference_integral_torque[:]=0.
                        self.status('CAPTURED',True)
                        return
                    self.swing_time=now
                    self.swing_from=self.q_est.copy()
                    if abs(float(wrap_delta(self.swing_from[1])))>math.radians(50.):
                        self.abort('UNSUPPORTED_INITIAL_PITCH_INVERSION');return
                    self.swing_to=self.swing_from.copy()
                    self.swing_to[1]=0.
                    self.swing_to[2]=self.swing_from[2]-float(wrap_delta(self.swing_from[2]))
                    self.yaw_hold=float(self.swing_from[0])
                    self.status('SWINGUP',True)
                return

            if self.state=='SWINGUP':
                elapsed=now-self.swing_time
                if elapsed>self.param('swingup_timeout_s'):
                    self.abort('SWINGUP_TIMEOUT');return
                s,sd,sdd=fifth_order(elapsed,self.param('swingup_duration_s'))
                dq=self.swing_to-self.swing_from
                target=self.swing_from+s*dq
                velocity=sd*dq
                accel=sdd*dq
                self.last_target=target.copy()
                tau=self.controller(target,velocity,accel)
                if not self.send_command(tau,dt,now,'SWINGUP'):return
                # NEVER switch to local LQR just because |angle| < 32 degrees.
                # The full trajectory includes deceleration to approximately
                # zero velocity at the upright setpoint.
                if elapsed>=self.param('swingup_duration_s'):
                    self.brake_time=now;self.brake_hold=None
                    self.status('BRAKING',True)
                return

            if self.state=='BRAKING':
                if now-self.brake_time>self.param('braking_timeout_s'):
                    self.abort('BRAKING_TIMEOUT');return
                target=np.array([self.yaw_hold,0.,0.])
                self.last_target=target.copy()
                tau=self.controller(target,np.zeros(3),np.zeros(3))
                tau[0]=np.clip(tau[0],-self.param('catch_yaw_torque_nm'),
                               self.param('catch_yaw_torque_nm'))
                if not self.send_command(tau,dt,now,'BRAKING'):return
                if self.local_condition(self.param('brake_angle_deg'),
                                        self.param('brake_rate_deg_s')):
                    if self.brake_hold is None:self.brake_hold=now
                    elif now-self.brake_hold>=self.param('brake_hold_s'):
                        self.catch_time=now
                        self.blend_time=now
                        self.yaw_hold=float(self.q_est[0])
                        self.yaw_stable_since=None
                        self.local_bad_since=None
                        self.status('CATCHING',True)
                else:
                    self.brake_hold=None
                return

            if self.state=='CATCHING':
                if now-self.catch_time>self.param('catch_timeout_s'):
                    self.abort('CATCH_TIMEOUT');return
                if not self.local_envelope_ok(now):
                    self.abort('CATCH_LOCAL_ESCAPE');return
                s,_,_=fifth_order(now-self.blend_time,self.param('lqr_blend_duration_s'))
                target=np.array([self.yaw_hold,0.,0.])
                self.last_target=target.copy()
                tau=self.blended_controller(target,np.zeros(3),np.zeros(3),s)
                tau[0]=np.clip(tau[0],-self.param('catch_yaw_torque_nm'),
                               self.param('catch_yaw_torque_nm'))
                if not self.send_command(tau,dt,now,'CATCHING'):return
                if (now-self.catch_time>=self.param('catch_min_hold_s')
                    and self.local_condition(self.param('yaw_start_angle_deg'),
                                             self.param('yaw_start_rate_deg_s'))):
                    if self.yaw_stable_since is None:self.yaw_stable_since=now
                    elif now-self.yaw_stable_since>=self.param('capture_hold_s'):
                        self.yaw_start=float(self.q_est[0])
                        self.yaw_end=self.yaw_start-float(wrap_delta(self.yaw_start))
                        self.yaw_elapsed=0.;self.yaw_align_time=now
                        self.status('YAW_ALIGN',True)
                else:self.yaw_stable_since=None
                return

            if self.state=='YAW_ALIGN':
                if now-self.yaw_align_time>self.param('yaw_align_timeout_s'):
                    self.abort('YAW_ALIGN_TIMEOUT');return
                if not self.local_envelope_ok(now):
                    self.abort('YAW_ALIGN_LOCAL_ESCAPE');return
                # Stop advancing the yaw trajectory if maintaining pitch/roll
                # becomes difficult. This reserves actuator authority for balance.
                horizontal_stable=self.local_condition(
                    self.param('yaw_motion_angle_deg'),
                    self.param('yaw_motion_rate_deg_s'))
                if horizontal_stable:
                    self.yaw_elapsed=min(self.param('yaw_return_duration_s'),
                                         self.yaw_elapsed+dt)
                s,sd,sdd=fifth_order(self.yaw_elapsed,self.param('yaw_return_duration_s'))
                dyaw=self.yaw_end-self.yaw_start
                target=np.array([self.yaw_start+s*dyaw,0.,0.])
                velocity=np.array([sd*dyaw if horizontal_stable else 0.,0.,0.])
                acceleration=np.array([sdd*dyaw if horizontal_stable else 0.,0.,0.])
                self.last_target=target.copy()
                tau=self.controller(target,velocity,acceleration,use_lqr=True)
                tau[0]=np.clip(tau[0],-self.param('yaw_align_torque_nm'),
                               self.param('yaw_align_torque_nm'))
                if not self.send_command(tau,dt,now,'YAW_ALIGN'):return
                near=(self.yaw_elapsed>=self.param('yaw_return_duration_s')
                    and self.local_condition(self.param('capture_roll_pitch_deg'),
                                             self.param('capture_rate_deg_s'))
                    and abs(math.degrees(wrap_delta(self.q_est[0])))<self.param('capture_yaw_deg')
                    and abs(math.degrees(self.qd_est[0]))<self.param('capture_rate_deg_s'))
                if near:
                    if self.cross_hold is None:self.cross_hold=now
                    elif now-self.cross_hold>=self.param('capture_hold_s'):
                        self.local_bad_since=None
                        self.filtered_q_ref[:]=0.
                        self.filtered_q_ref_velocity[:]=0.
                        self.filtered_q_ref_acceleration[:]=0.
                        self.reference_integral_torque[:]=0.
                        self.status('CAPTURED',True)
                else:self.cross_hold=None
                return

            # CAPTURED: local LQR, gravity compensation and explicit envelope.
            if not self.local_envelope_ok(now):
                self.abort('LEFT_LOCAL_ENVELOPE');return
            target=self.update_filtered_reference(dt)
            if target is None:return
            self.last_target=target.copy()
            # Keep the same URDF-based LQR + gravity compensation that was
            # already working. Only the small, smoothly changing setpoint moves.
            tau=self.controller(target,self.filtered_q_ref_velocity,
                                self.filtered_q_ref_acceleration,use_lqr=True)
            tau+=self.captured_integral_correction(target,dt)
            if self.fault:return
            tau[0]=np.clip(tau[0],-self.param('yaw_align_torque_nm'),
                           self.param('yaw_align_torque_nm'))
            self.send_command(tau,dt,now,'CAPTURED')
            return
        if self.armed:
            self.status('WAIT_ACTUATOR_DISARM');return
        near=(max(abs(np.degrees(wrap_delta(self.q_est[1:]))))<self.param('capture_roll_pitch_deg')
              and abs(math.degrees(wrap_delta(self.q_est[0])))<self.param('capture_yaw_deg')
              and np.max(abs(np.degrees(self.qd_est)))<self.param('capture_rate_deg_s'))
        if lqr and not near:
            self.cross_hold=None;self.status('WAIT_MANUAL_POSITION');return
        if lqr:
            if self.cross_hold is None:self.cross_hold=now
            if now-self.cross_hold<self.param('capture_hold_s'):
                self.status('CAPTURE_HOLD');return
        self.arm_pub.publish(Bool(data=True))
        self.arm_time=now;self.active_strategy=self.strategy
        self.command[:]=0.
        self.status('SWINGUP_ARMING' if swing else 'ARMING',True)


def main():
    rclpy.init()
    node=None
    try:
        node=BalanceNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.stop('SHUTDOWN')
            node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()