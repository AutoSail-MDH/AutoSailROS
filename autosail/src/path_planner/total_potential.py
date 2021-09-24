import numpy as np
from path_planner import potential_relative_goal_obsticle as prgo
from path_planner import potential_relative_wind as prw
from path_planner import create_profile as cp
import matplotlib.pyplot as plt


def calculate_total_potential(profile, list_len, obstacle, obstacle_weight, d_inf, goal, goal_weight, p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, heading):
    """
    Summarizes all the potentials in one point
    :param profile: The points in the area being considered
    :param list_len: Length of the profile.
    :param obstacle: A numpy array of all the obstacles.
    :param obstacle_weight: The fixed weight value of the obstacle.
    :param d_inf: The zone of influence for obstacles.
    :param goal: The postion of the goal i.e current waypoint
    :param goal_weight: The fixed weight value of the goal.
    :param p_ngz: The fixed value added to a points potential if it is in the no-go zone
    :param p_hyst: The value added to the potential of a point if the point is behind the vessel i.e the coast of
     tacking.
    :param g_v: A set scalar for the velocity of the vessel
    :param v_v: The velocity of the vessel
    :param w_speed: The wind speed
    :param w_theta: The wind angle
    :param heading: The heading of the vessel
    :return: The profile were every point contations it total potential.
    """
    p = np.array([0, 0])
    for i in range(list_len + 1):
        p[0] = profile[i].g_kx
        p[1] = profile[i].g_ky
        u_o = prgo.potential_relative_obstacle_calculation(obstacle, obstacle_weight, d_inf, p)
        u_g = prgo.potential_relative_goal_calculation(goal, goal_weight, p)
        p[0] = profile[i].l_kx
        p[1] = profile[i].l_ky
        u_w = prw.wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
        profile[i].u = u_o + u_g + u_w
    return profile


def reshape_profile(profile, dim):
    """
    Rechapes the porifle into a matrix for the heat map plot.
    :param profile: The points in the area being considered
    :param dim: The lenght of one side in the square matrix around the vessel.
    :return: The profile as a matrix.
    """
    g = 0
    if (dim % 2) == 0:
        dim = dim + 1
    profile_matrix = np.zeros((dim, dim))
    for j in range(dim):
        for i in range(dim):
            profile_matrix[dim - 1 - i, j] = profile[g].u
            g = g + 1
    return profile_matrix


def find_global_minima_angle(profile):
    """
    Finds the minimum potential in the profile.
    :param profile:The points in the area being considered
    :return: The angle and index to/of the minimum potential in the profile.
    """
    min_index = find_global_minima_index(profile)
    angle = np.arctan2(profile[min_index].l_ky, profile[min_index].l_kx)
    return angle, min_index


def find_global_minima_index(profile):
    """
    Findes the index for the minimum potential
    :param profile:The points in the area being considered
    :return: The index of the minimum potential
    """
    potential = [obj.u for obj in profile]
    return potential.index(min(potential))


if __name__ == '__main__':
    # Profile parameters
    diameter = 100
    position_v = np.array([50, 50])
    # Goal & obstacle parameters
    obstacle = np.array([[20, 10], [50, 10], [60, 80]])
    goal = np.array([70, 70])
    goal_weight = 4
    obstacle_weight = 100
    d_inf = 10
    # Wind parameters
    heading = np.array([0, 1])
    w_speed = 10
    w_theta = 0
    p_ngz = 20
    p_hyst = 5
    g_v = 1
    v_v = 4

    profile, list_len = cp.create_profile(dim=diameter, pos_v=position_v)
    profile = calculate_total_potential(profile, list_len, obstacle, obstacle_weight, d_inf, goal, goal_weight, p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, heading)
    profile_matrix = reshape_profile(profile, diameter)

    find_global_minima_angle(profile)

    plt.imshow(profile_matrix, cmap='hot', interpolation='nearest')
    plt.show()
