from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import reference_envelope as re

class IdentityModel:
    def estimate_joints(self, R, guess):
        roll,pitch,yaw=Rotation.from_matrix(R).as_euler('xyz')
        return np.array([yaw,pitch,roll]),0.,1.
    def mass_gravity(self, q):
        return np.eye(3),np.zeros(3)
    def motor_effectiveness(self, q, kf, km):
        return np.zeros((3,4))

limits, report = re.estimate_reference_envelope(
    IdentityModel(), np.eye(3), [30.,30.,180.], [180.5,30.,30.],
    4700.,5400.,[.4,1.25,1.25],.22,1.5e-7,1.e-8)
print('Identity kinematics:',limits,report)
assert report['valid'] and np.allclose(limits,[30,30,180]), 'Full-range kinematics test failed'
class RestrictedModel(IdentityModel):
    def estimate_joints(self,R,guess):
        q,res,cnd=super().estimate_joints(R,guess)
        if max(abs(q[1]),abs(q[2])) > np.deg2rad(16.):
            cnd=100.
        return q,res,cnd
limits, report = re.estimate_reference_envelope(
    RestrictedModel(), np.eye(3), [30.,30.,180.], [180.5,30.,30.],
    4700.,5400.,[.4,1.25,1.25],.22,1.5e-7,1.e-8)
print('Restricted kinematics:',limits,report)
assert np.all(limits[:2] <= 16.) and limits[2] >= 179.,'Bounded test failed'
print('Offline model-envelope tests PASS')
