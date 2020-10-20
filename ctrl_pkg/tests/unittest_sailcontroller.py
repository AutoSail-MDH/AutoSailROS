import unittest
import sys
import numpy as np

from ctrl.sail_controller import sail_angle_calculation
#  from ctrl import rudder_controller
sys.path.append('../scripts/')


class TestSailController(unittest.TestCase):

    #  to sail_angle_calculation position 0 takes wind speed, 1 takes wind angle
    def test_negative_angle(self):
        self.assertAlmostEqual(sail_angle_calculation([5, -45]), 0.3020762166913262)
        self.assertEqual(sail_angle_calculation([5, -90]), 0)
        self.assertAlmostEqual(sail_angle_calculation([5, -180]), -0.6041524333826525)

    def test_positive_angle(self):
        self.assertAlmostEqual(sail_angle_calculation([5, 45]), -0.3020762166913262)
        self.assertEqual(sail_angle_calculation([5, 90]), 0)
        self.assertAlmostEqual(sail_angle_calculation([5, 180]), 0.6041524333826525)


#  class TestPID(unittest.TestCase):
#    def test_course_straight(self):
#        return
    

if __name__ == '__main__':
    unittest.main()