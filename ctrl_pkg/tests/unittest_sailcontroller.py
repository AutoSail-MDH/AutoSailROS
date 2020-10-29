#!/usr/bin/env python
import unittest
import sys

from ctrl.sail_controller import sail_angle_calculation
#  from ctrl import rudder_controller
PKG = 'package'


class TestSailController(unittest.TestCase):

    def test_negative_angle(self):
        sail_limits = 0.60415243338
        self.assertAlmostEqual(sail_angle_calculation(-45, sail_limits), 0.30207621669)
        self.assertEqual(sail_angle_calculation(-90, sail_limits), 0)
        self.assertAlmostEqual(sail_angle_calculation(-180, sail_limits), -0.60415243338)

    def test_positive_angle(self):
        sail_limits = 0.60415243338
        self.assertAlmostEqual(sail_angle_calculation(45, sail_limits), -0.30207621669)
        self.assertEqual(sail_angle_calculation(90, sail_limits), 0)
        self.assertAlmostEqual(sail_angle_calculation(180, sail_limits), 0.60415243338)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("ctrl_pkg", "unittest_sailcontroller", TestSailController)
