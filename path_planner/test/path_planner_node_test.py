import numpy as np
from path_planner import *

import unittest


class TestStringMethods(unittest.TestCase):

    def test_upper(self):
        obstacle = np.array([])
        obstacle_weight = 0
        d_inf = 0
        p = np.array([0, 0])
        u_o = prgo.potential_relative_obstacle_calculation(obstacle, obstacle_weight, d_inf, p)
        self.assertEqual(u_o, 0)

        obstacle = np.array([[2, 2]])
        obstacle_weight = 100
        d_inf = 0
        p = np.array([1, 1])
        u_o = prgo.potential_relative_obstacle_calculation(obstacle, obstacle_weight, d_inf, p)
        self.assertEqual(u_o, 0)

        goal = np.array([4, 4])
        goal_weight = 0
        p = np.array([1, 1])
        u_g = prgo.potential_relative_goal_calculation(goal, goal_weight, p)
        self.assertEqual(u_g, 0)

if __name__ == '__main__':
    unittest.main()