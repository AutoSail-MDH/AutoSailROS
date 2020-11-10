#!/usr/bin/env python
import unittest

import rospy
import math
from ctrl.sail_controller import calculate_sail_angle
from ctrl.sail_controller import trim_sail


class TestSailController(unittest.TestCase):

    def test_negative_angle(self):
        sail_limits = math.pi/1.4
        max_roll = math.pi/6
        self.assertAlmostEqual(calculate_sail_angle(-max_roll, max_roll, sail_limits), 0)
        self.assertEqual(calculate_sail_angle(-max_roll/2, max_roll, sail_limits), 0)
        self.assertAlmostEqual(calculate_sail_angle(0, max_roll, sail_limits), 0)

    def test_positive_angle(self):
        sail_limits = math.pi/1.4
        max_roll = math.pi / 6
        self.assertAlmostEqual(calculate_sail_angle(max_roll, max_roll, sail_limits), sail_limits)
        self.assertEqual(calculate_sail_angle(max_roll/2, max_roll, sail_limits), sail_limits/2)
        self.assertAlmostEqual(calculate_sail_angle(0, max_roll, sail_limits), 0)

    def sail_trimming_test(self):
        sail_limits = math.pi / 1.4
        servo_scalar = rospy.get_param("~sail_servo_scalar", 20.25)
        self.assertEqual(trim_sail(sail_angle=0, sail_limits=sail_limits, scalar=servo_scalar), 1620)
        self.assertEqual(trim_sail(sail_angle=80, sail_limits=sail_limits, scalar=servo_scalar), 0)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("ctrl_pkg", "unittest_sailcontroller", TestSailController)
