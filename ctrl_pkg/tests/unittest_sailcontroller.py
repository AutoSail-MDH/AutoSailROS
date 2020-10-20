import unittest
import sys
import numpy as np

from ctrl.sail_controller import sail_angle_calculation
#  from ctrl import rudder_controller
sys.path.append('../scripts/')


class TestSailController(unittest.TestCase):

    #  to sail_angle_calculatipon position 0 takes wind speed, 1 takes wind angle
    def test_negative_angle(self):
        self.assertAlmostEqual(self, sail_angle_calculation([5, 90]), -0)

    def test_positive_angle(self):
        self.assertEqual(self, sail_angle_calculation([5, -90]), -0)


#  class TestPID(unittest.TestCase):
#    def test_course_straight(self):
#        return
    

if __name__ == '__main__':
    unittest.main()