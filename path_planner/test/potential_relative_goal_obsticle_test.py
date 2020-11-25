#!/usr/bin/env python
import numpy as np
from path_planner import *
from path_planner import potential_relative_goal_obsticle as prgo
from path_planner import potential_field_algorithm as pfa

import unittest


class TestPotentialRelativeGoalObstacle(unittest.TestCase):

    def test_parematers_work_correctly(self):
        dim = 10
        potential_field_object = pfa.PotentialField(dim, 0, 0, 0, 0, 0, 0)
        obstacle = np.array([])
        p = np.array([0, 0])
        u_o = potential_field_object._potential_relative_obstacle_calculation(obstacle, p)
        self.assertEqual(u_o, 0)

        obstacle = np.array([[2, 2]])
        p = np.array([1, 1])
        u_o = potential_field_object._potential_relative_obstacle_calculation(obstacle, p)
        self.assertEqual(u_o, 0)

        goal = np.array([4, 4])
        p = np.array([1, 1])
        u_g = potential_field_object._potential_relative_goal_calculation(goal, p)
        self.assertEqual(u_g, 0)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("path_planner", "potential_relative_goal_obsticle_test", TestPotentialRelativeGoalObstacle)

    #  unittest.main()