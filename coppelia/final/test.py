from tools import launcher
from robot import robot
from tools import locomotion
import unittest
import numpy as np
from math import pi
import time

class Test(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.motor_names = ['Omnirob_FLwheel_motor', 'Omnirob_FRwheel_motor', 'Omnirob_RRwheel_motor', 'Omnirob_RLwheel_motor']
        cls.r = robot('Omnirob', cls.motor_names)  # Create an instance of our robot

    @classmethod
    def tearDownClass(cls):
        cls.r.close_connection()

        
    def test_tf3d(self):
        expected = np.array([0,1,1])
        actual = launcher.transform3d(np.array([1,0,1]), rotation=pi/2)
        self.assertTrue(np.allclose(actual, expected))
    def test_get_launch_params(self):
        v_0, theta_0 = launcher.get_launch_params(self.r)
        self.assertTrue(v_0 > 0 and theta_0 > 0)
    def test_fire(self):
        target = self.r.get_object_position('Bill_head')
        robo_coords = self.r.get_position(-1)
        dx = target[0] - robo_coords[0]
        dy = target[1] - robo_coords[1]
        gamma_d = -np.arctan2(dx, dy)
        locomotion.rotate_to(gamma_d, self.r)
        v, theta = launcher.get_launch_params(self.r)
        print(f'v_0, theta_0: {v, theta}')
        launcher.fire(v, theta, self.r)
    def test_prismatic(self):
        self.r.set_joint_velocity('launcher_prismatic_joint', 5.94)
    def test_revolute(self):
        joint_name = 'launcher_revolute_joint'
        target_orientation = pi/4
        self.r.set_joint_position(joint_name, target_orientation)
        time.sleep(2)
        actual = self.r.get_joint_position(joint_name)
        # Control loop has error ~ 1 degree ~ 0.022 radians
        self.assertAlmostEqual(actual, target_orientation, delta=0.03)
        

if __name__ == "__main__":
    unittest.main()