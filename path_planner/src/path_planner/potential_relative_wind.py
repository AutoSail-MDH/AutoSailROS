import numpy as np
import math
from path_planner import speed_polar_diagram_function as spdf


def wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading):
    [max_vel, up_beat, dn_beat, w_theta] = spdf.speed_polar_diagram_calculation(w_speed, w_theta)
    no_go = np.array([np.deg2rad(up_beat), np.deg2rad(dn_beat)])
    w_theta = np.deg2rad(w_theta)
#    print("no-go", no_go)
#    print("point angle", np.arctan2(p[1], p[0]))
#    print("w_theta", w_theta)

    point_angle = np.arctan2(p[1], p[0])
    heading_angle = np.arctan2(heading[1], heading[0])
    while heading_angle < 0:
        heading_angle = heading_angle + np.pi*2
#    print("heading_angle", heading_angle)
    rel_heading_angle = heading_angle - w_theta
#    print("rel_heading_angle", rel_heading_angle)
    rel_point_angle = point_angle - w_theta
#    print("rel_point_angle", rel_point_angle)
#   self.error = math.atan2(math.sin(self.setpoint-control_signal), math.cos(self.setpoint-control_signal))
    while  rel_point_angle < 0:
        rel_point_angle = rel_point_angle + 2*np.pi
#        print("rel_point_angle2", rel_point_angle)

    while rel_heading_angle < 0:
#       if rel_heading_angle < 0:
        rel_heading_angle = rel_heading_angle + 2*np.pi
#        print("rel_heading_angle2", rel_heading_angle)

    while rel_point_angle > 2*np.pi:
        rel_point_angle = rel_point_angle - 2*np.pi
#        print("rel_point_angle3", rel_point_angle)

    if rel_heading_angle > 2*np.pi:
        rel_heading_angle = rel_heading_angle - 2*np.pi
#        print("rel_heading_angle3", rel_point_angle)

#    print("rel_point_angleF", rel_point_angle)
#    print("rel_heading_angleF", rel_heading_angle)
#    if abs(rel_point_angle) > np.pi:
#        rel_point_angle = rel_point_angle + np.pi*2
#        print("rel_point_angle_update", rel_point_angle)

    if (no_go[1] <= rel_point_angle <= no_go[1] + (np.pi - no_go[1])) or (no_go[0] >= abs(rel_point_angle) and abs(rel_point_angle) >= 0) \
            or (abs(rel_point_angle) >= (2*np.pi - no_go[0])):
        if no_go[1] <= rel_point_angle <= no_go[1] + (np.pi - no_go[1]):
            print("case1.1")
        elif no_go[0] >= abs(rel_point_angle) and abs(rel_point_angle) >= 0:
            print("case1.2")
        elif abs(rel_point_angle) >= (2*np.pi - no_go[0]):
            print("case1.3", 2*np.pi - no_go[0])
        return "case1"
#        return p_ngz
#    print("calc", (np.pi - rel_point_angle + no_go[0]))
    if (rel_heading_angle < no_go[1] and rel_point_angle > no_go[1]) or (rel_heading_angle > no_go[1] and rel_point_angle < no_go[1]):
        if (rel_heading_angle < no_go[1] and rel_point_angle > no_go[1]):
            print("case2.1")
        else:
            print("case2.2")
        return "case2"
#        return p_hyst + g_v * ((v_v - max_vel) / max_vel)
    else:
        return "case3"
#        return g_v * ((v_v - max_vel) / max_vel)


if __name__ == '__main__':
    w_speed = 4
    w_theta = 270
    p_ngz = 20
    p_hyst = 5
    g_v = 1
    v_v = 1
    heading = np.array([1, 0])
    p = np.array([2, 1])
    u_w = wind_potential_calculation(p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, p, heading)
    print(u_w)







