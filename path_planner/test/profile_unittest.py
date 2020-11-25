#!/usr/bin/env python
import numpy as np
from path_planner import *
from path_planner import potential_field_algorithm as pfa
import math
import unittest


class TestProfile(unittest.TestCase):

    def test_check_if_even_statement(self):
        dim = 6
        pos_v = [0, 0]
        potential_field_object = pfa.PotentialField(dim, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        profile, list_len = potential_field_object._create_profile(pos_v=pos_v)
        self.assertAlmostEqual(round(np.sqrt(list_len)), dim + 1, 1)

        dim = 4
        pos_v = [0, 0]
        potential_field_object = pfa.PotentialField(dim, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        profile, list_len = potential_field_object._create_profile(pos_v=pos_v)
        self.assertAlmostEqual(round(np.sqrt(list_len)), dim + 1, 1)

        dim = 34
        pos_v = [0, 0]
        potential_field_object = pfa.PotentialField(dim, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        profile, list_len = potential_field_object._create_profile(pos_v=pos_v)
        self.assertAlmostEqual(round(np.sqrt(list_len)), dim + 1, 1)

    def test_vessel_pos_in_profile(self):
        dim = 5
        pos_v = [5, 3]
        potential_field_object = pfa.PotentialField(dim, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        profile, list_len = potential_field_object._create_profile(pos_v=pos_v)
        self.assertAlmostEqual(profile[math.floor(list_len/2)].l_kx, 0)
        self.assertAlmostEqual(profile[math.floor(list_len / 2)].l_ky, 0)
        self.assertAlmostEqual(profile[math.floor(list_len / 2)].g_kx, 5)
        self.assertAlmostEqual(profile[math.floor(list_len / 2)].g_ky, 3)

        dim = 10
        pos_v = [5, 3]
        potential_field_object = pfa.PotentialField(dim, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        profile, list_len = potential_field_object._create_profile(pos_v=pos_v)
        self.assertAlmostEqual(profile[math.floor(list_len / 2)].l_kx, 0)
        self.assertAlmostEqual(profile[math.floor(list_len / 2)].l_ky, 0)
        self.assertAlmostEqual(profile[math.floor(list_len / 2)].g_kx, 5)
        self.assertAlmostEqual(profile[math.floor(list_len / 2)].g_ky, 3)

        dim = 13
        pos_v = [6, 2]
        potential_field_object = pfa.PotentialField(dim, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        profile, list_len = potential_field_object._create_profile(pos_v=pos_v)
        self.assertAlmostEqual(profile[math.floor(list_len / 2)].l_kx, 0)
        self.assertAlmostEqual(profile[math.floor(list_len / 2)].l_ky, 0)
        self.assertAlmostEqual(profile[math.floor(list_len / 2)].g_kx, 6)
        self.assertAlmostEqual(profile[math.floor(list_len / 2)].g_ky, 2)

    def test_vessel_min1_pos_in_profile(self):
        dim = 5
        pos_v = [5, 3]
        potential_field_object = pfa.PotentialField(dim, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        profile, list_len = potential_field_object._create_profile(pos_v=pos_v)
        self.assertAlmostEqual(profile[math.floor(list_len / 2) - 1].l_kx, 0)
        self.assertAlmostEqual(profile[math.floor(list_len / 2) - 1].l_ky, -1)
        self.assertAlmostEqual(profile[math.floor(list_len / 2) - 1].g_kx, 5)
        self.assertAlmostEqual(profile[math.floor(list_len / 2) - 1].g_ky, 2)

    def test_vessel_min2_pos_in_profile(self):
        dim = 5
        pos_v = [5, 3]
        potential_field_object = pfa.PotentialField(dim, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        profile, list_len = potential_field_object._create_profile(pos_v=pos_v)
        self.assertAlmostEqual(profile[math.floor(list_len / 2) - 2].l_kx, 0)
        self.assertAlmostEqual(profile[math.floor(list_len / 2) - 2].l_ky, -2)
        self.assertAlmostEqual(profile[math.floor(list_len / 2) - 2].g_kx, 5)
        self.assertAlmostEqual(profile[math.floor(list_len / 2) - 2].g_ky, 1)

    def test_vessel_plus1_pos_in_profile(self):
        dim = 5
        pos_v = [5, 3]
        potential_field_object = pfa.PotentialField(dim, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        profile, list_len = potential_field_object._create_profile(pos_v=pos_v)
        self.assertAlmostEqual(profile[math.floor(list_len / 2) + 1].l_kx, 0)
        self.assertAlmostEqual(profile[math.floor(list_len / 2) + 1].l_ky, 1)
        self.assertAlmostEqual(profile[math.floor(list_len / 2) + 1].g_kx, 5)
        self.assertAlmostEqual(profile[math.floor(list_len / 2) + 1].g_ky, 4)

    def test_vessel_plus2_pos_in_profile(self):
        dim = 5
        pos_v = [5, 3]
        potential_field_object = pfa.PotentialField(dim, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        profile, list_len = potential_field_object._create_profile(pos_v=pos_v)
        self.assertAlmostEqual(profile[math.floor(list_len / 2) + 2].l_kx, 0)
        self.assertAlmostEqual(profile[math.floor(list_len / 2) + 2].l_ky, 2)
        self.assertAlmostEqual(profile[math.floor(list_len / 2) + 2].g_kx, 5)
        self.assertAlmostEqual(profile[math.floor(list_len / 2) + 2].g_ky, 5)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("path_planner", "profile_unittest", TestProfile)
    #  unittest.main()
