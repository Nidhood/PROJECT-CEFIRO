#!/usr/bin/env python3
"""Static, model-based SIM attitude reference-envelope estimator.

This estimator is deliberately conservative.  Feasible *static* equilibria and
well-conditioned joint kinematics do NOT prove LQR stability or transient
tracking.  Full experimental validation in Gazebo is required before expanding
operating limits, especially for simultaneous roll/pitch commands.
"""
from __future__ import annotations

import math
import numpy as np
from scipy.spatial.transform import Rotation

from cefiro_multibody import wrap_delta, allocate_motors


def estimate_reference_envelope(
    model,
    body_nominal_rotation,
    requested_limits_deg,
    joint_limits_deg,
    nominal_rpm,
    max_rpm,
    max_torque_nm,
    allocation_residual_nm,
    thrust_coeff,
    reaction_coeff,
    angle_step_deg=2.0,
):
    """Return [roll, pitch, yaw] max magnitudes and a detailed analysis dict.

    Checks both signs of each axis, the four horizontal corners and center,
    and the dynamics' zero-velocity feedforward torque (gravity compensation).
    The yaw range represents all headings over [-180,180], with shortest-path
    rotations; it does not imply a commanded 360-degree physical spin.
    """
    requested = np.asarray(requested_limits_deg, dtype=float)
    joint = np.asarray(joint_limits_deg, dtype=float)
    torque_max = np.asarray(max_torque_nm, dtype=float)
    if (requested.shape != (3,) or joint.shape != (3,) or
            torque_max.shape != (3,) or not np.all(np.isfinite(requested)) or
            not np.all(np.isfinite(joint)) or np.any(requested <= 0) or
            np.any(joint <= 0) or np.any(torque_max <= 0)):
        raise ValueError('Reference-envelope parameters must be finite positive 3-vectors')
    requested = np.minimum(requested, [30.0, 30.0, 180.0])
    # The actual local-escape threshold is 35 deg in joint pitch/roll.  Keep
    # at least 5 deg of model-reference margin even if config is permissive.
    joint = np.minimum(joint, [180.5, 30.0, 30.0])
    kf, km = float(thrust_coeff), float(reaction_coeff)
    nominal = float(nominal_rpm)
    maximum = float(max_rpm)
    max_residual = float(allocation_residual_nm)
    if kf <= 0 or km <= 0 or maximum <= nominal or nominal <= 0 or max_residual <= 0:
        raise ValueError('Invalid motor/torque parameters for reference analysis')

    results = {}
    def feasible(rpy_deg):
        key = tuple(round(float(v), 4) for v in rpy_deg)
        if key in results:
            return results[key][0]
        # Kinematics from the same CAD model used by the simulation controller.
        R_des = body_nominal_rotation @ Rotation.from_euler(
            'xyz', np.radians(rpy_deg)).as_matrix()
        q, residual_rad, condition = model.estimate_joints(
            R_des, np.zeros(3, dtype=float))
        q = wrap_delta(np.asarray(q, dtype=float))
        valid = (np.all(np.isfinite(q)) and
                 np.isfinite(residual_rad) and np.isfinite(condition) and
                 residual_rad <= math.radians(0.3) and condition <= 10.0 and
                 np.all(np.abs(np.degrees(q)) <= joint + 1.e-3))
        diagnostic = dict(joints_deg=np.degrees(q).tolist(),
                          ik_residual_deg=float(math.degrees(residual_rad)),
                          ik_condition=float(condition))
        if valid:
            _, gravity = model.mass_gravity(q)
            gravity = np.asarray(gravity, dtype=float)
            diagnostic['gravity_torque_nm'] = gravity.tolist()
            # Leave torque headroom for acceleration, perturbation rejection
            # and integral action; equilibrium feasibility is not enough.
            valid = (np.all(np.isfinite(gravity)) and
                     np.all(np.abs(gravity) <= 0.9 * torque_max))
            if valid:
                G = model.motor_effectiveness(q, kf, km)
                _, residual, _ = allocate_motors(
                    G, gravity, nominal, maximum,
                    axis_weights=(0.35, 1.0, 1.0))
                diagnostic['allocation_residual_nm'] = float(residual)
                valid = bool(np.isfinite(residual) and
                             residual <= 0.9 * max_residual)
        results[key] = (bool(valid), diagnostic)
        return bool(valid)

    if not feasible((0.0, 0.0, 0.0)):
        # An infeasible equilibrium is a serious model/configuration mismatch.
        # Do not invent an apparently safe larger operating range.
        return np.array([5.0, 5.0, 10.0]), {
            'valid': False, 'reason': 'Nominal equilibrium infeasible in static CAD/ESC analysis',
            'equilibrium': results[(0.0, 0.0, 0.0)][1], 'checks': len(results)}

    step = max(0.5, float(angle_step_deg))
    def contiguous_limit(axis):
        cap = float(requested[axis]); last = 0.0
        # Always check intermediate orientations rather than jumping to the cap.
        n = max(1, math.ceil(cap / (step if axis < 2 else 5.0)))
        for j in range(1, n+1):
            value = min(cap, j*cap/n)
            good = True
            for sign in (-1.0, 1.0):
                cmd = np.zeros(3); cmd[axis] = sign * value
                good &= feasible(cmd)
            if not good:
                break
            last = value
        return last

    limits = np.array([contiguous_limit(i) for i in range(3)], dtype=float)
    independent_limits = limits.copy()

    # Individual axes may be feasible while their combined corners are not.
    # Find the largest uniform contraction that keeps the 3x3 horizontal grid.
    a, b = limits[:2]
    for scale in np.linspace(1.0, 0.0, 41):
        rr, pp = float(a*scale), float(b*scale)
        if all(feasible((r, p, 0.0))
               for r in (-rr, 0.0, rr) for p in (-pp, 0.0, pp)):
            limits[0], limits[1] = rr, pp
            break
    # No dynamically plausible nonzero displacement was found for at least
    # one axis. Keep original pilot limits, and clearly report the failure.
    if np.any(limits < [0.5, 0.5, 0.5]):
        return np.array([5.0, 5.0, 10.0]), dict(
            valid=False, reason='No valid nonzero model envelope on all axes',
            independent_limits_deg=independent_limits.tolist(),
            evaluated_limits_deg=limits.tolist(), checks=len(results))
    limits[:2] = np.floor(limits[:2] * 2.0) / 2.0
    limits[2] = math.floor(limits[2] * 10.0) / 10.0
    # Preserve the physical 0..180 heading interpretation, not multirevolution.
    return limits, dict(valid=True, independent_limits_deg=independent_limits.tolist(),
                        final_limits_deg=limits.tolist(), checks=len(results),
                        caveat='Static feasibility only; dynamic Gazebo tests mandatory')
