#!/usr/bin/env python
import unittest
import sys

from ctrl.sail_controller import sail_angle_calculation
#  from ctrl import rudder_controller
PKG = 'package'
sys.path.append('../scripts/')


class TestSailController(unittest.TestCase):

    #  to sail_angle_calculation position 0 takes wind speed, 1 takes wind angle
    def test_negative_angle(self):
        sail_limits = [-0.60415243338, 0.60415243338]
        self.assertAlmostEqual(sail_angle_calculation(-45, sail_limits), 0.30207621669)
        self.assertEqual(sail_angle_calculation(-90, sail_limits), 0)
        self.assertAlmostEqual(sail_angle_calculation(-180, sail_limits), -0.60415243338)

    def test_positive_angle(self):
        sail_limits = [-0.60415243338, 0.60415243338]
        self.assertAlmostEqual(sail_angle_calculation(45, sail_limits), -0.30207621669)
        self.assertEqual(sail_angle_calculation(90, sail_limits), 0)
        self.assertAlmostEqual(sail_angle_calculation(180, sail_limits), 0.60415243338)


#  class TestPID(unittest.TestCase):
#    def test_course_straight(self):
#        return
    

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, "unittest_sail_controller", TestSailController)
