#!/usr/bin/env python
import unittest

from ctrl.sail_controller import calculate_sail_angle
from ctrl.sail_controller import trim_sail


class TestSailController(unittest.TestCase):

    def test_negative_angle(self):
        sail_limits = 0.60415243338
        self.assertAlmostEqual(calculate_sail_angle(-45, sail_limits), 0.30207621669)
        self.assertEqual(calculate_sail_angle(-90, sail_limits), 0)
        self.assertAlmostEqual(calculate_sail_angle(-180, sail_limits), -0.60415243338)

    def test_positive_angle(self):
        sail_limits = 0.60415243338
        self.assertAlmostEqual(calculate_sail_angle(45, sail_limits), -0.30207621669)
        self.assertEqual(calculate_sail_angle(90, sail_limits), 0)
        self.assertAlmostEqual(calculate_sail_angle(180, sail_limits), 0.60415243338)

    def sail_trimming_test(self):
        self.assertEqual(trim_sail(0), 1620)
        self.assertEqual(trim_sail(80), 0)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("ctrl_pkg", "unittest_sailcontroller", TestSailController)
