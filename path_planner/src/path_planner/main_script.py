import numpy as np
from path_planner import potential_relative_goal_obsticle as prgo
from path_planner import potential_relative_wind as prw
from path_planner import create_profile as cp
from path_planner import total_potential as tp
import matplotlib.pyplot as plt

def main_loop( diameter, position_v, obstacle, obstacle_weight, d_inf, goal, goal_weight, p_ngz, p_hyst, g_v,
              v_v, w_speed, w_theta, heading):
    '''
    The function calculates and saves the global x,y coordinates for all points the vessel travels through to one
    waypoint.
    :param diameter: The lenght of one side in the square matrix around the vessel for which potential is calculated.
    :param position_v: The current position of the vessel.
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
    :return: The x,y coordinates for all points from the start of the calculation to the goal.
    '''
    i = 0
    x =[]
    y = []
    x.append(position_v[0])
    y.append(position_v[1])
    while 1:
        if np.linalg.norm(position_v-goal) < 5:
            return x, y, i
        profile, list_len = cp.create_profile(dim=diameter, pos_v=position_v)
        profile = tp.calculate_total_potential(profile, list_len, obstacle, obstacle_weight, d_inf, goal, goal_weight,
                                               p_ngz, p_hyst, g_v, v_v, w_speed, w_theta, heading)
        min_angle, min_index = tp.find_global_minima_angle(profile)
        x.append(profile[min_index].g_kx)
        y.append(profile[min_index].g_ky)
        position_v[0] = profile[min_index].g_kx
        position_v[1] = profile[min_index].g_ky
        heading[0] = profile[min_index].l_kx
        heading[1] = profile[min_index].l_ky
        i = i + 1

def calc_all_waypoints(diameter, position_v, obstacle, obstacle_weight, d_inf, goal, goal_weight, p_ngz, p_hyst, g_v,
              v_v, w_speed, w_theta, heading, length):
    """
    Calculates the main_loop function for all waypoints and saves the whole path in an array.
    :param diameter: The lenght of one side in the square matrix around the vessel for which potential is calculated.
    :param position_v: The current position of the vessel.
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
    :param heading: The heading of the vesse
    :param length: An array with the lengths of the segments of the path.
    :return: The x,y coordinates for all points from the start of the path planner to the end goal.
    """
    for j in range(len(waypoints) ):
        goal = waypoints[j]
        x_j, y_j, i = main_loop(diameter, position_v, obstacle, obstacle_weight, d_inf, goal, goal_weight, p_ngz, p_hyst, g_v,
              v_v, w_speed, w_theta, heading)
        length[j+1] = len(x_j) + length[j]
        x[int(length[j]):int(length[j+1])] = x_j
        y[int(length[j]):int(length[j+1])] = y_j
    return x, y

if __name__ == '__main__':
    # Profile parameters
    diameter = 10
    # Goal & obstacle parameters
    goal_weight = 4
    obstacle_weight = 200
    d_inf = 40
    # Wind parameters
    heading = np.array([1, 0])
    w_speed = 10
    # theta
    w_theta = 90
    p_ngz = 40
    p_hyst = 16
    g_v = 1
    v_v = 4
    x = []
    y = []
    obstacle = np.array([[0, 110], [0, 90]])
    waypoints = np.array([[200, 100], [100, 200], [100, 100], [0, 100]])
    goal = waypoints[0]
    position_v = np.array([300, 100])
    start = np.copy(position_v)

    length = np.zeros(len(waypoints) + 1)
    x, y = calc_all_waypoints(diameter, position_v, obstacle, obstacle_weight, d_inf, goal, goal_weight, p_ngz, p_hyst,
                              g_v,
                              v_v, w_speed, w_theta, heading, length)

    plt.plot(x, y)

    plt.plot(obstacle[0][0], obstacle[0][1], 'x')
    plt.plot(obstacle[1][0], obstacle[1][1], 'x')
    plt.plot(waypoints[0][0], waypoints[0][1], 'o')
    plt.plot(waypoints[1][0], waypoints[1][1], 'o')
    plt.plot(waypoints[2][0], waypoints[2][1], 'o')
    plt.plot(waypoints[3][0], waypoints[3][1], 'o')


    plt.xlabel('x')
    plt.ylabel('y')
    plt.title(
        "Wind " + str(w_theta) + '°' + ', start (' + str(start[0]) + ',' + str(start[1]) + ')' + ', hystersis ' + str(
            p_hyst))

    plt.show()

""" Orginal 

    obstacle = np.array([[40, 40], [50, 60], [83, 94], [82, 100], [0, 80], [45, 35], [50, 30], [115, 70]])
    waypoints = np.array([[100, 100], [200, 200], [200, 100], [150, 50]])
    goal = waypoints[0]
    position_v = np.array([0, 0])
    start = np.copy(position_v)

    length = np.zeros(len(waypoints)+1)
    x, y = calc_all_waypoints(diameter, position_v, obstacle, obstacle_weight, d_inf, goal, goal_weight, p_ngz, p_hyst, g_v,
              v_v, w_speed, w_theta, heading, length)

    plt.plot(x, y)

    plt.plot(obstacle[0][0], obstacle[0][1], 'x')
    plt.plot(obstacle[1][0], obstacle[1][1], 'x')
    plt.plot(obstacle[2][0], obstacle[2][1], 'x')
    plt.plot(obstacle[3][0], obstacle[3][1], 'x')
    plt.plot(obstacle[4][0], obstacle[4][1], 'x')
    plt.plot(obstacle[5][0], obstacle[5][1], 'x')
    plt.plot(obstacle[6][0], obstacle[6][1], 'x')
    plt.plot(obstacle[7][0], obstacle[7][1], 'x')
    plt.plot(waypoints[0][0], waypoints[0][1], 'o')
    plt.plot(waypoints[1][0], waypoints[1][1], 'o')
    plt.plot(waypoints[2][0], waypoints[2][1], 'o')
    plt.plot(waypoints[3][0], waypoints[3][1], 'o')

    plt.xlabel('x')
    plt.ylabel('y')
    plt.title("Wind " + str(w_theta) + '°' + ', start (' + str(start[0]) + ',' + str(start[1]) + ')' + ', hystersis ' 
    + str(p_hyst))

    plt.show()
    
    """