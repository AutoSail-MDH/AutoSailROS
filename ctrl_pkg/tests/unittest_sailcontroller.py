#!/usr/bin/env python
import unittest

import numpy as np
from ctrl.sail_controller import calculate_sail_angle
from ctrl.sail_controller import trim_sail


class TestSailController(unittest.TestCase):

    def test_negative_angle(self):
        sail_limits = np.pi/5.2
        self.assertAlmostEqual(calculate_sail_angle(-np.pi/4, sail_limits), sail_limits/2)
        self.assertEqual(calculate_sail_angle(-np.pi/2, sail_limits), 0)
        self.assertAlmostEqual(calculate_sail_angle(-np.pi, sail_limits), -sail_limits)

    def test_positive_angle(self):
        sail_limits = np.pi/5.2
        self.assertAlmostEqual(calculate_sail_angle(np.pi/4, sail_limits), -sail_limits/2)
        self.assertEqual(calculate_sail_angle(np.pi/2, sail_limits), 0)
        self.assertAlmostEqual(calculate_sail_angle(np.pi, sail_limits), sail_limits)

    def sail_trimming_test(self):
        self.assertEqual(trim_sail(0), 1620)
        self.assertEqual(trim_sail(80), 0)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("ctrl_pkg", "unittest_sailcontroller", TestSailController)
