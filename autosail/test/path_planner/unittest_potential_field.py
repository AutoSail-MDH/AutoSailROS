import math
import numpy as np
from unittest import TestCase
import rosunit
from path_planner.potential_field_algorithm import PotentialField


class TestPotentialField(TestCase):

    def setUp(self):
        self.pf = PotentialField(50, 100, 20, 1, 50, 10, 10)

    def test_calc_heading(self):
        position = [0, 0]
        heading = [1, 0]
        velocity = 4
        goal = [50, 0]
        wind_speed = 3 * 1.94384
        wind_theta = 0
        course = self.pf.calc_heading(goal, heading, wind_speed, wind_theta, position, [], velocity)
        self.pf.plot_heat_map(2, heading)
        self.assertLess(abs(45 - abs(math.degrees(course))), 5)

        wind_speed = 5 * 1.94384
        wind_theta = math.pi/2
        course = self.pf.calc_heading(goal, heading, wind_speed, wind_theta, position, [], velocity)
        self.pf.plot_heat_map(2, heading)
        self.assertLess(abs(math.degrees(course)), 5)

    def test_potential_relative_obstacle(self):
        dim = 10
        obstacle = np.array([])
        p = np.array([0, 0])
        u_o = self.pf._potential_relative_obstacle_calculation(obstacle, p)
        self.assertEqual(u_o, 0)

        obstacle = np.array([[2, 2]])
        p = np.array([2, 2])
        u_o = self.pf._potential_relative_obstacle_calculation(obstacle, p)
        self.assertEqual(u_o, 100)
        p = np.array([1, 1])
        u_o = self.pf._potential_relative_obstacle_calculation(obstacle, p)
        self.assertLess(u_o, 100)

    def test_potential_relative_goal(self):
        goal = np.array([4, 4])
        p = np.array([4, 4])
        u_g = self.pf._potential_relative_goal_calculation(goal, p)
        self.assertEqual(u_g, 0)
        p = np.array([3, 4])
        u_g = self.pf._potential_relative_goal_calculation(goal, p)
        self.assertEqual(u_g, 1)

    def test_speed_polar_diagram(self):
        w_speed = 1
        w_theta = 90
        [max_vel, up_beat, dn_beat, w_theta] = self.pf._speed_polar_diagram_calculation(w_speed, w_theta)
        self.assertEqual([max_vel, up_beat, dn_beat], [4.78, 45, 139.4])

        w_speed = 30
        w_theta = 30
        [max_vel, up_beat, dn_beat, w_theta] = self.pf._speed_polar_diagram_calculation(w_speed, w_theta)
        self.assertEqual([max_vel, up_beat, dn_beat], [12.6, 32.6, 148.1])

        w_speed = 4.6
        w_theta = 350
        [max_vel, up_beat, dn_beat, w_theta] = self.pf._speed_polar_diagram_calculation(w_speed, w_theta)
        self.assertEqual([max_vel, up_beat, dn_beat], [4.78, 45, 139.4])

        w_speed = -4.6
        w_theta = 350
        [max_vel, up_beat, dn_beat, w_theta] = self.pf._speed_polar_diagram_calculation(w_speed, w_theta)
        self.assertEqual([max_vel, up_beat, dn_beat], [4.78, 45, 139.4])


if __name__ == "__main__":
    rosunit.unitrun("autosail", "potential_field", TestPotentialField)