import numpy as np
import math
from path_planner import speed_polar_diagram_function as spdf


def wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading):
    """
    Calculates the potential in one point relative the wind.
    :param p_ngz: The fixed value added to a points potential if it is in the no-go zone
    :param p_hyst: The value added to the potential of a point if the point is behind the vessel i.e the coast of
     tacking.
     :param g_v: A set scalar for the velocity of the vessel
    :param v_v: The velocity of the vessel
    :param w_speed: The wind speed
    :param w_theta: The wind angle
    :param p: The x,y coordinate of the point being calculated.
    :param heading: The heading of the vessel
    :return: The potential in one point relative the wind.
    """
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
            or (abs(rel_point_angle) >= (2 * np.pi - no_go[0])):
        return "case1"
#        return p_ngz
    if (rel_heading_angle < no_go[1] < rel_point_angle) or (
            rel_heading_angle > no_go[1] > rel_point_angle):
        return "case2"
#        return p_hyst + g_v * ((v_v - max_vel) / max_vel)
    else:
        return "case3"
#        return g_v * ((v_v - max_vel) / max_vel)


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
