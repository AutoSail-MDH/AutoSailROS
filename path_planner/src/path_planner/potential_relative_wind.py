import numpy as np
import math
from path_planner import speed_polar_diagram_function as spdf


def wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading):
    [max_vel, up_beat, dn_beat, w_theta] = spdf.speed_polar_diagram_calculation(w_speed, w_theta)
    no_go = np.array([np.deg2rad(up_beat), np.deg2rad(dn_beat)])
    w_theta = np.deg2rad(w_theta)
    point_angle = np.arctan2(p[1], p[0])
    heading_angle = np.arctan2(heading[1], heading[0])
    rel_heading_angle = heading_angle - w_theta
    rel_point_angle = point_angle - w_theta

    while rel_point_angle < 0:
        rel_point_angle = rel_point_angle + 2 * np.pi
    while rel_point_angle > 2 * np.pi:
        rel_point_angle = rel_point_angle - 2 * np.pi

    while rel_heading_angle < 0:
        rel_heading_angle = rel_heading_angle + 2 * np.pi
    if rel_heading_angle > 2 * np.pi:
        rel_heading_angle = rel_heading_angle - 2 * np.pi



    if (no_go[1] <= rel_point_angle <= no_go[1] + 2*(np.pi - no_go[1])) or (
            no_go[0] >= abs(rel_point_angle) >= 0) \
            or (abs(rel_point_angle) >= (2 * np.pi - no_go[0])): #(2 * np.pi - no_go[0]))
        # print("case1")
        return p_ngz
    if (rel_heading_angle < no_go[1] < rel_point_angle) or (
            rel_heading_angle > no_go[1] > rel_point_angle):
        # print("case2")
        return p_hyst + g_v * ((v_v - max_vel) / max_vel)
    else:
        # print("case3")
        #        print("max_vel", max_vel)
        #        print("v_v", v_v)
        return g_v * ((v_v - max_vel) / max_vel)


if __name__ == '__main__':
    w_speed = 4
    w_theta = 0
    p_ngz = 20
    p_hyst = 5
    g_v = 1
    v_v = 1
    heading = np.array([1, 0])
    p = np.array([1, -2])
    u_w = wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
    print(u_w)
