#!/usr/bin/env python
import unittest

import math
from ctrl.sail_controller_wind_angle import calculate_sail_angle, trim_sail


class TestSailController(unittest.TestCase):

    def test_starboard_wind(self):
        sail_limits = math.radians(80)
        max_roll = math.pi/6
        # wind from -pi/4
        self.assertAlmostEqual(calculate_sail_angle(0, max_roll, -(1/4)*math.pi, sail_limits), sail_limits)
        # wind from -pi/2
        self.assertAlmostEqual(calculate_sail_angle(0, max_roll, -(2/4)*math.pi, sail_limits), math.pi/4)
        # wind from -(3/4)*pi
        self.assertAlmostEqual(calculate_sail_angle(0, max_roll, -(3/4)*math.pi, sail_limits), 0)

    def test_port_wind(self):
        sail_limits = math.radians(80)  # 2.24399
        max_roll = math.pi / 6
        # wind from pi/4
        self.assertAlmostEqual(calculate_sail_angle(0, max_roll, (1/4)*math.pi, sail_limits), -sail_limits)
        # wind from pi/2
        self.assertAlmostEqual(calculate_sail_angle(0, max_roll, (2/4)*math.pi, sail_limits), -math.pi/4)
        # wind from (3/4)*pi
        self.assertAlmostEqual(calculate_sail_angle(0, max_roll, (3/4)*math.pi, sail_limits), 0)

    def test_negative_roll_correction(self):
        sail_limits = math.radians(80)
        max_roll = math.pi/6
        wind_angle = math.pi
        self.assertAlmostEqual(calculate_sail_angle(-max_roll, max_roll, wind_angle, sail_limits), -sail_limits)
        self.assertAlmostEqual(calculate_sail_angle(-max_roll/2, max_roll, wind_angle, sail_limits), -sail_limits/2)
        self.assertAlmostEqual(calculate_sail_angle(0, max_roll, wind_angle, sail_limits), 0)

    def test_positive_roll_correction(self):
        sail_limits = math.radians(80)
        max_roll = math.pi / 6
        wind_angle = math.pi
        self.assertAlmostEqual(calculate_sail_angle(max_roll, max_roll, wind_angle, sail_limits), sail_limits)
        self.assertAlmostEqual(calculate_sail_angle(max_roll/2, max_roll, wind_angle, sail_limits), sail_limits/2)
        self.assertAlmostEqual(calculate_sail_angle(0, max_roll, wind_angle, sail_limits), 0)

    def test_roll_and_wind(self):
        sail_limits = math.radians(80)
        max_roll = math.pi / 6

        wind_angle = math.pi/2
        desired_roll = -math.pi/12
        self.assertAlmostEqual(calculate_sail_angle(desired_roll, max_roll, wind_angle, sail_limits), -sail_limits)

        wind_angle = (3/4)*math.pi
        desired_roll = -math.pi/12
        self.assertAlmostEqual(calculate_sail_angle(desired_roll, max_roll, wind_angle, sail_limits), -sail_limits/2)

    def test_sail_trimming(self):
        sail_limits = math.radians(80)
        self.assertEqual(trim_sail(sail_angle=0, sail_limits=sail_limits), 1620)
        self.assertEqual(trim_sail(sail_angle=sail_limits, sail_limits=sail_limits), 0)
        self.assertEqual(trim_sail(sail_angle=-sail_limits, sail_limits=sail_limits), 0)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("autosail", "unittest_sailcontroller", TestSailController)
