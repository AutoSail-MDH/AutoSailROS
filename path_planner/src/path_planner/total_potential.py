import numpy as np
from path_planner import potential_relative_goal_obsticle as prgo
from path_planner import potential_relative_wind as prw
from path_planner import create_profile as cp
import matplotlib.pyplot as plt


def calculate_total_potential(profile, list_len, obstacle, obstacle_weight, d_inf, goal, goal_weight, p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, heading):
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
    min_index = find_global_minima_index(profile)
    angle = np.arctan2(profile[min_index].l_ky, profile[min_index].l_kx)
    print(angle)


def find_global_minima_index(profile):

    potential =[obj.u for obj in profile]
    return potential.index(min(potential))
#    print(min(potential), potential.index(min(potential)))
#    print(potential[potential.index(min(potential))])

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
