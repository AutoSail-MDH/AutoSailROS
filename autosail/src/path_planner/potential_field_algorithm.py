import numpy as np
import matplotlib.pyplot as plt


class PointClass:
    def __init__(self, l_kx, l_ky, g_kx, g_ky, u):
        self.l_kx = l_kx
        self.l_ky = l_ky
        self.g_kx = g_kx
        self.g_ky = g_ky
        self.u = u


class PotentialField:
    # angles for the no-go zones depending on wind speed
    up_beat = np.array([45, 42.4, 40.5, 37.5, 35.4, 34.1, 33.5, 32.8, 32.6])
    dn_beat = np.array([139.4, 143.1, 146.4, 152.4, 163.7, 169.2, 170.8, 169.7, 148.1])
    #        angles = np.array([32, 36, 40, 45, 60, 70, 80, 90, 100, 110, 120, 130, 135, 140, 150, 160, 170, 180])
    speed_array = np.array([4, 6, 8, 10, 12, 14, 16, 20, 25])
    # the potential velocity of the vessel depending on wind speed (x) and wind angle (y)
    speed_diagram = np.array([[2.69, 4.03, 5.11, 5.89, 6.35, 6.61, 6.77, 6.98, 7.15],
                              [3.06, 4.5, 5.62, 6.38, 6.74, 6.94, 7.08, 7.28, 7.44],
                              [3.39, 4.9, 6.05, 6.72, 7, 7.18, 7.31, 7.51, 7.68],
                              [3.74, 5.31, 6.44, 7.01, 7.26, 7.42, 7.55, 7.75, 7.93],
                              [4.47, 6.1, 6.99, 7.5, 7.78, 7.95, 8.1, 8.36, 8.62],
                              [4.71, 6.32, 7.12, 7.64, 8.01, 8.22, 8.42, 8.72, 9.04],
                              [4.78, 6.36, 7.14, 7.66, 8.11, 8.48, 8.69, 9.06, 9.52],
                              [4.7, 6.35, 7.35, 7.81, 8.06, 8.51, 8.9, 9.44, 10.05],
                              [4.71, 6.5, 7.4, 7.92, 8.21, 8.46, 8.82, 9.79, 10.65],
                              [4.7, 6.44, 7.3, 7.89, 8.35, 8.65, 8.91, 9.62, 11.25],
                              [4.47, 6.17, 7.1, 7.71, 8.28, 8.79, 9.15, 9.88, 11.01],
                              [4.02, 5.69, 6.79, 7.45, 8.01, 8.59, 9.19, 10.39, 11.74],
                              [3.76, 5.4, 6.6, 7.28, 7.84, 8.4, 8.98, 10.53, 12.19],
                              [3.5, 5.1, 6.37, 7.11, 7.65, 8.19, 8.74, 10.24, 12.6],
                              [2.98, 4.45, 5.71, 6.66, 7.25, 7.74, 8.23, 9.4, 11.78],
                              [2.53, 3.82, 5.01, 6.06, 6.83, 7.38, 7.85, 8.84, 10.77],
                              [2.3, 3.48, 4.6, 5.63, 6.5, 7.11, 7.6, 8.53, 10.09],
                              [2.16, 3.28, 4.35, 5.35, 6.25, 6.91, 7.4, 8.29, 9.62]])

    def __init__(self, diameter, obstacle_weight, d_inf, goal_weight, p_ngz, p_hyst, g_v, wind_limit):
        """

        :param diameter: diameter of profile
        :param obstacle_weight: weight of obstacles when calculating potential
        :param d_inf: distance of influence for the obstacles
        :param goal_weight: weight of goal when calculating potential
        :param p_ngz: potential for no go zone
        :param p_hyst: added potential for hysteris points
        :param g_v: scalar weight for the effect of the velocity of the vessel
        """
        self.diameter = diameter
        self.obstacle_weight = obstacle_weight
        self.d_inf = d_inf
        self.goal_weight = goal_weight
        self.p_ngz = p_ngz
        self.p_hyst = p_hyst
        self.g_v = g_v
        self.profile = None
        self.wind_limit = wind_limit

        plt.ion()

    def find_nearest(self, array, value):
        """
        round wind speed to closest hole number 4,6,8,10,12,14,16,20,25
        :param array: The array of values i.e 4,6,8,10,12,14,16,20,25
        :param value: The value being rounded
        :return: The rounded wind speed
        """
        array = np.asarray(array)
        idx = (np.abs(array - value)).argmin()
        return array[idx]

    def _speed_polar_diagram_calculation(self, w_theta, w_speed):
        """
        Calculates the maximum velocity and no-go zones.
        :param w_theta: Wind angle
        :return: Maximum possible velocity, no-go zones and wind angle.
        """
        w_speed_polar_diagram = w_speed
        # wind speed < 4 set to 4
        if w_speed_polar_diagram < 4:
            w_speed_polar_diagram = 4
        # winds peed > 25 set to 25
        if w_speed_polar_diagram > 25:
            w_speed_polar_diagram = 25
        # round wind speed to closest hole number 4,6,8,10,12,14,16,20,25
        w_speed_polar_diagram = self.find_nearest(self.speed_array, w_speed_polar_diagram)
        # remap the wind speed to the corresponding index in the arrays.
        w_speed_index = np.where(
            self.speed_array == w_speed_polar_diagram)
        w_speed_index = w_speed_index[0]
        # return the max velocity, no-go zones and wind angle
        max_vel = max(np.amax(self.speed_diagram[:, w_speed_index], axis=1))
        return [max_vel, max(self.up_beat[w_speed_index]), max(self.dn_beat[w_speed_index]), w_theta]

    def _create_profile(self, pos_v):
        """
        Creates a list for all points in determined area around the boat with  local, global coordinates and the
         potential for that points.
        :param pos_v: The position of the vessel
        :return: A list for all points in determined area around the boat with  local, global coordinates and the
         potential for that points.
        """
        dim = self.diameter
        profile_ = []
        # ensures that the dimension of the profile is odd
        if (dim % 2) == 0:
            dim = dim + 1
        radius = (dim - 1) / 2
        l_range = np.linspace(-radius, radius, dim)
        list_len = dim * dim - 1
        zero = np.array((0, 0))
        p_d = np.array((0, 0))
        # create the profile as a list with PointClass at every index
        for j in range(dim):
            for i in range(dim):
                p_d[0] = -radius + j
                p_d[1] = l_range[i]
                if np.linalg.norm(zero - p_d) <= radius:
                    p = PointClass(-radius + j, l_range[i], pos_v[0] - radius + j, pos_v[1] - radius + i, 0)
                    profile_.append(p)
                else:
                    p = PointClass(-radius + j, l_range[i], pos_v[0] - radius + j, pos_v[1] - radius + i, 100)
                    profile_.append(p)

        return profile_, list_len

    def _potential_relative_obstacle_calculation_j(self, obstacle, p):
        """
        Calculates the potential for a single point relative one obstacle.
        :param obstacle: x,y coordinates for one obstacle.
        :param p: The x,y coordinate of the point being calculated.
        :return: The potential for one point relative obstacles.
        """
        if np.linalg.norm(p - obstacle) <= self.d_inf:
            if p[0] == obstacle[0] and p[1] == obstacle[1]:
                return self.obstacle_weight
            else:
                return self.obstacle_weight * ((1 / np.linalg.norm(p - obstacle)) - (1 / self.d_inf))
        elif np.linalg.norm(p - obstacle) > self.d_inf:
            return 0

    def _potential_relative_obstacle_calculation(self, obstacle, p):
        """
        A summation of the potential in one point for all the obstacles.
        :param obstacle: x,y coordinates for one obstacle.
        :param p: The x,y coordinate of the point being calculated.
        :return: The summation of the potential in one point for all the obstacles.
        """
        u_o = 0
        obstacle_shape = obstacle.shape
        for i in range(obstacle_shape[0]):
            u_o = self._potential_relative_obstacle_calculation_j(obstacle[i, :], p) + u_o
        return u_o

    def _potential_relative_goal_calculation(self, goal, p):
        """
        The potential for one point relative the goal.
        :param goal: x,y coordinate for hte goal.
        :param p: The x,y coordinate of the point being calculated.
        :return: The potential in one point relative the goal.
        """
        return self.goal_weight * np.linalg.norm(p - goal)

    def _wind_potential_calculation(self, w_theta, p, heading, w_speed, v_v):
        """
        Calculates the potential for a point relative the wind
        :param w_theta: Wind angle
        :param p: current point
        :param heading: Heading of the vessel
        :return: the correct potential of the point
        """
        # calculate max velocity, no-go zones and wind angle
        [max_vel, up_beat, dn_beat, w_theta] = self._speed_polar_diagram_calculation(w_theta, w_speed)
        no_go = np.array([np.deg2rad(up_beat), np.deg2rad(dn_beat)])
        # calculates the angle of the point and the heading
        point_angle = np.arctan2(p[1], p[0])
        heading_angle = np.arctan2(heading[1], heading[0])
        # calculate the angle of the point and the heading relative the wind angle
        rel_heading_angle = heading_angle - w_theta
        rel_point_angle = point_angle - w_theta
        # ensures the relative angle of the point is in the interval 0:2pi
        while rel_point_angle < 0:
            rel_point_angle = rel_point_angle + 2 * np.pi
        while rel_point_angle > 2 * np.pi:
            rel_point_angle = rel_point_angle - 2 * np.pi
        # ensures the relative angle of the heading is in the interval 0:2pi
        while rel_heading_angle < 0:
            rel_heading_angle = rel_heading_angle + 2 * np.pi
        if rel_heading_angle > 2 * np.pi:
            rel_heading_angle = rel_heading_angle - 2 * np.pi
        # checks three cases and calculates the corresponding potential:
        # 1. the point is in the no-go zones
        # 2. the point is not in the no-go zones and the heading is in the other zone of  direction as the point
        # 3. the point is not in the no-go zones and the heading is in the same zone of  direction as the point
        if (no_go[1] <= rel_point_angle <= no_go[1] + 2 * (np.pi - no_go[1])) or (
                no_go[0] >= abs(rel_point_angle) >= 0) \
                or (abs(rel_point_angle) >= (2 * np.pi - no_go[0])):
            return self.p_ngz
        if (rel_heading_angle < no_go[1] < rel_point_angle) or (
                rel_heading_angle > no_go[1] > rel_point_angle):
            return self.p_hyst + self.g_v * ((v_v - max_vel) / max_vel)
        else:
            return self.g_v * ((v_v - max_vel) / max_vel)

    def _calculate_total_potential(self, profile, list_len, obstacle, goal, w_theta, heading, w_speed, v_v):
        """
        Summarizes all the potentials in one point
        :param profile: The points in the area being considered
        :param list_len: Length of the profile.
        :param obstacle: A numpy array of all the obstacles.
        :param goal: The postion of the goal i.e current waypoint
        :param w_theta: The wind angle
        :param heading: The heading of the vessel
        :return: The profile were every point contains it total potential.
        """
        p = np.array([0, 0])
        # calculates and sum the potential relative obstacle, goal and wind for all points in the profile
        for i in range(list_len + 1):
            p[0] = profile[i].g_kx
            p[1] = profile[i].g_ky
            u_o = self._potential_relative_obstacle_calculation(obstacle, p)
            u_g = self._potential_relative_goal_calculation(goal, p)
            p[0] = profile[i].l_kx
            p[1] = profile[i].l_ky
            if w_speed > self.wind_limit:
                u_w = self._wind_potential_calculation(w_theta, p, heading, w_speed, v_v)
                profile[i].u = u_o + u_g + u_w + profile[i].u
            else:
                profile[i].u = u_o + u_g + profile[i].u
        return profile

    def _reshape_profile(self, profile):
        """
        Rechapes the profile into a matrix for the heat map plot.
        :param profile: The points in the area being considered
        :return: The profile as a matrix.
        """
        g = 0
        if (self.diameter % 2) == 0:
            self.diameter = self.diameter + 1
        profile_matrix = np.zeros((self.diameter, self.diameter))
        for j in range(self.diameter):
            for i in range(self.diameter):
                profile_matrix[self.diameter - 1 - i, j] = profile[g].u
                g = g + 1
        profile_matrix_noflip = profile_matrix
        profile_matrix = np.flip(np.flip(np.transpose(profile_matrix), 0), 1)
        return profile_matrix

    def _find_global_minima_angle(self, profile):
        """
        Finds the minimum potential in the profile.
        :param profile:The points in the area being considered
        :return: The angle and index to/of the minimum potential in the profile.
        """
        min_index = self._find_global_minima_index(profile)
        angle = np.arctan2(profile[min_index].l_ky, profile[min_index].l_kx)
        return angle, min_index

    def _find_global_minima_index(self, profile):
        """
        Finds the index for the minimum potential
        :param profile:The points in the area being considered
        :return: The index of the minimum potential
        """
        potential = [obj.u for obj in profile]
        return potential.index(min(potential))

    def _calculate_profile(self, position_v, obstacles_array, goal, w_theta, heading, w_speed, v_v):
        """
        creates the profile and calculates the desired heading
        :param w_theta: wind angle
        :param position_v: the position of the vessel
        :param obstacles_array: an array of the xy coordinates for the obstacle
        :param goal: xy position of the goal
        :return: minimum angle and the profile
        """
        # creates a profile around square around the position of the vessel in which potential will be calculated
        profile, list_len = self._create_profile(pos_v=position_v)
        # calculates the total potential in each point
        profile = self._calculate_total_potential(profile, list_len, obstacles_array, goal, w_theta,
                                                  heading, w_speed, v_v)
        # calculates the index of the position in the profile with the lowest potential and the angle from the vessel
        # position to that point
        min_angle, min_index = self._find_global_minima_angle(profile)
        return min_angle, profile

    def plot_heat_map(self, sleep=0.01, heading=None):
        """
        method used to plot the heat map
        profile: profile for the vessel
        """
        if self.profile is not None:
            plt.clf()
            profile_matrix = self._reshape_profile(self.profile)
            plt.imshow(profile_matrix, cmap='hot', interpolation='nearest')
            if heading is not None:
                heading_x = np.linspace(self.diameter / 2 - 1 / 2, self.diameter / 2 + heading[1] * 25, 10)
                heading_y = np.linspace(self.diameter / 2 - 1 / 2, self.diameter / 2 - heading[0] * 25, 10)
                plt.plot(heading_x, heading_y, linewidth=3)
            plt.gcf().canvas.draw_idle()
            plt.gcf().canvas.start_event_loop(0.3)

    def reshpe_profile_2d_plot(self):
        profile_matrix = self._reshape_profile(self.profile)
        return profile_matrix

    def calc_heading(self, goal, heading, w_speed, w_theta, position_v, obstacles, v_v):
        """
        calculates the desired heading of the vessel
        :param heading: heading of vessel
        :param w_speed: wind speed
        :param v_v: vessel velocity
        :param obstacles: array of obstacles
        :param position_v: position of vessel
        :param w_theta: angle of the wind
        :param goal: position of gaol
        :return: desired heading
        """
        # create potential field object
        goal_pos = goal[0:2]
        position_v = [int(round(position_v[0])), int(round(position_v[1]))]
        # calculates the local x,y for the obstacles in the frame of the reference points
        obstacles = np.array(obstacles).astype(int)
        min_angle, profile = self._calculate_profile(position_v, obstacles, goal_pos,
                                                     w_theta, heading, w_speed, v_v)
        self.profile = profile

        return min_angle
