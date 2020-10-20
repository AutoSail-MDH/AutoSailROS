import numpy as np
from path_planner import *
from path_planner import speed_polar_diagram_function as spdf
import unittest

class TestStringMethods(unittest.TestCase):

    def test_upper(self):
        w_speed = 1
        w_theta = 90
        [max_vel, up_beat, dn_beat, w_theta] = spdf.speed_polar_diagram_calculation(w_speed, w_theta)
        self.assertEqual([max_vel, up_beat, dn_beat], [4.78, 45, 139.4])
        print(max_vel)
        print(up_beat)
        print(dn_beat)

        w_speed = 30
        w_theta = 30
        [max_vel, up_beat, dn_beat, w_theta] = spdf.speed_polar_diagram_calculation(w_speed, w_theta)
        self.assertEqual([max_vel, up_beat, dn_beat], [12.6, 32.6, 148.1])
        print(max_vel)
        print(up_beat)
        print(dn_beat)

        w_speed = 4.6
        w_theta = 350
        [max_vel, up_beat, dn_beat, w_theta] = spdf.speed_polar_diagram_calculation(w_speed, w_theta)
        self.assertEqual([max_vel, up_beat, dn_beat], [4.78, 45, 139.4])
        print(max_vel)
        print(up_beat)
        print(dn_beat)

        w_speed = -4.6
        w_theta = 350
        [max_vel, up_beat, dn_beat, w_theta] = spdf.speed_polar_diagram_calculation(w_speed, w_theta)
        self.assertEqual([max_vel, up_beat, dn_beat], [4.78, 45, 139.4])

        print(max_vel)
        print(up_beat)
        print(dn_beat)


if __name__ == '__main__':
    unittest.main()
