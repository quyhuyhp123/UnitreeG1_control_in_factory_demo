import unittest,math
import numpy as np
from perception.camera_stabilizer import CameraStabilizer
CFG={'position_tau_s':.12,'height_tau_s':.25,'yaw_tau_s':.1,'max_position_lag_m':.08}
class StabilizerTests(unittest.TestCase):
    def test_level_pitch_and_orthonormal(self):
        f=CameraStabilizer(CFG)
        for yaw in np.linspace(-4,4,100):
            T=f.step([1,2,1.2],yaw,15,.02)
            np.testing.assert_allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-10)
            self.assertAlmostEqual(T[2,0],0)
            self.assertAlmostEqual((-T[:3,2])[2],-math.sin(math.radians(15)))
    def test_bounce_suppression_and_lag_bound(self):
        f=CameraStabilizer(CFG);raw=[];smooth=[]
        for i in range(500):
            p=np.array([i*.02*.3,0,1.2+.02*math.sin(i*.02*2*math.pi*2)])
            T=f.step(p,0,15,.02)
            self.assertLessEqual(np.linalg.norm(T[:3,3]-p),.080001)
            if i>100:raw.append(p[2]);smooth.append(T[2,3])
        self.assertLess(np.std(smooth),.4*np.std(raw))
    def test_wrap_and_reset(self):
        f=CameraStabilizer(CFG);f.step([0,0,1],math.radians(179),10,.02)
        f.step([0,0,1],math.radians(-179),10,.02)
        self.assertLess(abs(f.yaw-math.pi),.1)
        f.reset();T=f.step([5,5,1],0,10,.02);np.testing.assert_allclose(T[:3,3],[5,5,1])
