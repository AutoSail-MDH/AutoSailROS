import numpy as np
from path_planner import potential_field_algorithm as pfa
import matplotlib.pyplot as plt


def competition_configuration_1():
    # Profile parameters
    diameter_init = 10
    # Goal & obstacle parameters
    goal_weight_init = 4
    obstacle_weight_init = 200
    d_inf_init = 40
    # Wind parameters
    heading_init = np.array([1, 0])
    w_speed_init = 10
    # theta
    w_theta_init = 90
    p_ngz_init = 40
    p_hyst_init = 10
    g_v_init = 1
    v_v_init = 4

    obstacle_init = np.array([[-50, 110], [-50, 90]])
    waypoints_init = np.array([[200, 100], [100, 200], [0, 100], [-50, 100]])
    position_v_init = np.array([300, 100])
    start = np.copy(position_v_init)

    potential_field_object = pfa.PotentialField(diameter_init, obstacle_weight_init, d_inf_init, goal_weight_init,
                                                p_ngz_init, p_hyst_init, g_v_init, v_v_init, w_speed_init)
    x, y = calc_all_waypoints(position_v_init, obstacle_init, w_theta_init, heading_init,
                              waypoints_init, potential_field_object)
    plot_path(x, y, obstacle_init, waypoints_init, w_theta_init, start, p_hyst_init)


def configuration_1():
    # Profile parameters
    diameter_init = 10
    # Goal & obstacle parameters
    goal_weight_init = 4
    obstacle_weight_init = 200
    d_inf_init = 40
    # Wind parameters
    heading_init = np.array([1, 0])
    w_speed_init = 10
    # theta
    w_theta_init = 45
    p_ngz_init = 40
    p_hyst_init = 10
    g_v_init = 1
    v_v_init = 4

    obstacle_init = np.array([[-50, 110], [-50, 90]])
    waypoints_init = np.array([[100, 100], [200, 200], [300, 200], [300, 100]])
    position_v_init = np.array([0, 0])
    start = np.copy(position_v_init)


    potential_field_object = pfa.PotentialField(diameter_init, obstacle_weight_init, d_inf_init, goal_weight_init,
                                                p_ngz_init, p_hyst_init, g_v_init, v_v_init, w_speed_init)
    x, y = calc_all_waypoints(position_v_init, obstacle_init, w_theta_init, heading_init,
                              waypoints_init, potential_field_object)
    plot_path(x, y, obstacle_init, waypoints_init, w_theta_init, start, p_hyst_init)


def calc_all_waypoints(position_v, obstacle, w_theta, heading, waypoints, potential_field_object):
    """
    Calculates the main_loop function for all waypoints and saves the whole path in an array.
    :param potential_field_object: PotentialField object.
    :param position_v: The current position of the vessel.
    :param obstacle: A numpy array of all the obstacles.
    :param w_theta: The wind angle
    :param heading: The heading of the vessel
    :param waypoints: List of all the waypoints
    :return: The x,y coordinates for all points from the start of the path planner to the end goal.
    """
    x_all = []
    y_all = []
    length = np.zeros(len(waypoints) + 1)
    for j in range(len(waypoints)):
        goal = waypoints[j]
        x_j, y_j = potential_field_object.calculate_segment(position_v, obstacle, goal, w_theta, heading)
        length[j+1] = len(x_j) + length[j]
        x_all[int(length[j]):int(length[j+1])] = x_j
        y_all[int(length[j]):int(length[j+1])] = y_j
    return x_all, y_all


def plot_path(x, y, obstacle, waypoints, w_theta, start, p_hyst):
    plt.plot(x, y)
    for i in range(len(obstacle)):
        plt.plot(obstacle[i][0], obstacle[i][1], 'x')
    for i in range(len(waypoints)):
        plt.plot(waypoints[i][0], waypoints[i][1], 'o')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title(
        "Wind " + str(w_theta) + '°' + ', start (' + str(start[0]) + ',' + str(start[1]) + ')' + ', hystersis '
        + str(p_hyst))
    plt.show()


if __name__ == '__main__':
    configuration_1()
#    competition_configuration_1()
