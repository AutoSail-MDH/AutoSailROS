import unittest
import sys
import numpy as np
from ctrl_pkg.src.ctrl import rudder_controller
from ctrl_pkg.src.ctrl import sail_controller
sys.path.append('../scripts/')


rudder_controller.pid_()

class TestSailController(unittest.TestCase):

    #  to sail_angle_calculatipon position 0 takes wind speed, 1 takes wind angle
    def test_angle(self):
        self.assertEqual(self, sail_angle_calculation([5, 45], (2*np.pi)/5))

class TestPID(unittest.TestCase):
    def test_course_straight(self):
        return
    

if __name__ == '__main__':
    unittest.main()