import numpy as np
from path_planner import *
from path_planner import potential_relative_wind as prw
import unittest

class TestStringMethods(unittest.TestCase):
    p_ngz = 20
    p_hyst = 5
    g_v = 1
    v_v = 4
    w_speed = 4

    def test_case_1(self):
        p_ngz = 20
        p_hyst = 5
        g_v = 1
        v_v = 4
        w_speed = 4
        heading = np.array([1, 1])

        #   Test 1.1
        w_theta = 0
        p = np.array([1, 0])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case1", case)

        p = np.array([-1, 0])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case1", case)

        #   Test 1.2
        heading = np.array([-1, 0])
        w_theta = 90
        p = np.array([0, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case1", case)

        p = np.array([0, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case1", case)

        #   Test 1.3
        heading = np.array([0, 1])
        w_theta = 180
        p = np.array([1, 0])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case1", case)

        p = np.array([-1, 0])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case1", case)

        #   Test 1.4
        heading = np.array([-1, 0])
        w_theta = 270
        p = np.array([0, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case1", case)

        p = np.array([0, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case1", case)

        #   Test 1.5
        heading = np.array([-1, 1])
        w_theta = 45
        p = np.array([1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case1", case)

        p = np.array([-1, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case1", case)

        #   Test 1.6
        heading = np.array([-1, 1])
        w_theta = 225
        p = np.array([1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case1", case)

        p = np.array([-1, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case1", case)

        #   Test 1.7
        heading = np.array([1, 1])
        w_theta = 135
        p = np.array([-1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case1", case)

        p = np.array([1, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case1", case)

        #   Test 1.8
        heading = np.array([1, 1])
        w_theta = 315
        p = np.array([-1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case1", case)

        p = np.array([1, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case1", case)



    def test_case_2(self):
        p_ngz = 20
        p_hyst = 5
        g_v = 1
        v_v = 4
        w_speed = 4

        #   Test 2.1
        w_theta = 0
        heading = np.array([0, -1])
        p = np.array([1, 2])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        p = np.array([-1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        #   Test 2.2
        w_theta = 0
        heading = np.array([0, 1])
        p = np.array([1, -2])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        p = np.array([-1, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        #   Test 2.3
        w_theta = 90
        heading = np.array([1, 0])
        p = np.array([-2, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        p = np.array([-1, -1])  # [-1, -1] Som Ting Wong
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        #   Test 2.4
        w_theta = 90
        heading = np.array([-1, 0])
        p = np.array([2, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        p = np.array([1, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        #   Test 2.5
        w_theta = 180
        heading = np.array([0, -1])
        p = np.array([1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        p = np.array([-1, 2])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        #   Test 2.6
        w_theta = 180
        heading = np.array([0, 1])
        p = np.array([1, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        p = np.array([-1, -2])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        #   Test 2.7
        w_theta = 270
        heading = np.array([1, 0])
        p = np.array([-1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        p = np.array([-2, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        #   Test 2.8
        w_theta = 270
        heading = np.array([-1, 0])
        p = np.array([1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        p = np.array([2, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        #   Test 2.9
        w_theta = 45
        heading = np.array([1, 1])
        p = np.array([1, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        p = np.array([2, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        #   Test 2.10
        w_theta = 45
        heading = np.array([1, -1])
        p = np.array([-1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        p = np.array([-2, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        #   Test 2.11
        w_theta = 225
        heading = np.array([-1, 1])
        p = np.array([1, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        p = np.array([2, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        #   Test 2.12
        w_theta = 225
        heading = np.array([1, -1])
        p = np.array([-1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        p = np.array([-2, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        #   Test 2.13
        w_theta = 135
        heading = np.array([-1, -1])
        p = np.array([1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        p = np.array([2, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        #   Test 2.14
        w_theta = 135
        heading = np.array([1, 1])
        p = np.array([-1, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        p = np.array([-4, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        #   Test 2.15
        w_theta = 315
        heading = np.array([-1, -1])
        p = np.array([1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        p = np.array([2, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        #   Test 2.16
        w_theta = 315
        heading = np.array([1, 1])
        p = np.array([-1, 0])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

        p = np.array([-4, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case2", case)

    def test_case_3(self):
        p_ngz = 20
        p_hyst = 5
        g_v = 1
        v_v = 4
        w_speed = 4

        #   Test 3.1
        w_theta = 0
        heading = np.array([0, 1])
        p = np.array([1, 2])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        p = np.array([-1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        #   Test 3.2
        w_theta = 0
        heading = np.array([0, -1])
        p = np.array([1, -2])  #  case1
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        p = np.array([-1, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        #   Test 3.3
        w_theta = 90
        heading = np.array([1, 0])
        p = np.array([2, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        p = np.array([1, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        #   Test 3.4
        w_theta = 90
        heading = np.array([-1, 0])
        p = np.array([-2, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        p = np.array([-1, -1])  # [-1. -1] Som Ting Wong
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        #   Test 2.5
        w_theta = 180
        heading = np.array([0, -1])
        p = np.array([-1, -2])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        p = np.array([1, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        #   Test 2.6
        w_theta = 180
        heading = np.array([0, 1])
        p = np.array([1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        p = np.array([-1, 2])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        #   Test 2.7
        w_theta = 270
        heading = np.array([1, 0])
        p = np.array([1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        p = np.array([2, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        #   Test 2.8
        w_theta = 270
        heading = np.array([-1, 0])
        p = np.array([-1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        p = np.array([-2, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        #   Test 2.9
        w_theta = 45
        heading = np.array([1, 1])
        p = np.array([-1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        p = np.array([-2, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        #   Test 2.10
        w_theta = 45
        heading = np.array([1, -1])
        p = np.array([2, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        p = np.array([3, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        #   Test 2.11
        w_theta = 225
        heading = np.array([-1, 1])
        p = np.array([-1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        p = np.array([-2, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        #   Test 2.12
        w_theta = 225
        heading = np.array([1, -1])
        p = np.array([2, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        p = np.array([4, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        #   Test 2.13
        w_theta = 135
        heading = np.array([-1, -1])
        p = np.array([-1, -2])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        p = np.array([-4, -1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        #   Test 2.14
        w_theta = 135
        heading = np.array([1, 1])
        p = np.array([1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        p = np.array([2, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        #   Test 2.15
        w_theta = 315
        heading = np.array([-1, -1])
        p = np.array([-1, 0])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        p = np.array([-4, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        #   Test 2.14
        w_theta = 315
        heading = np.array([1, 1])
        p = np.array([1, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)

        p = np.array([2, 1])
        case = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        self.assertEqual("case3", case)





if __name__ == '__main__':
    unittest.main()
