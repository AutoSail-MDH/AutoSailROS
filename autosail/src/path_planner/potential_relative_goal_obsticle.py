import numpy as np


def potential_relative_obstacle_calculation_j(obstacle, obstacle_weight, d_inf, p):
    """
    Calculates the potential for a single point relative one obstacle.
    :param obstacle: x,y coordinates for one obstacle.
    :param obstacle_weight: The fixed weight of a obstacle.
    :param d_inf: The fixed zone of influence for a the obstacles.
    :param p: The x,y coordinate of the point being calculated.
    :return: The potential for one point relative obstacles.
    """
    if np.linalg.norm(p-obstacle) <= d_inf:
        return obstacle_weight*((1/np.linalg.norm(p-obstacle)) - (1/d_inf))
    elif np.linalg.norm(p-obstacle) > d_inf:
        return 0


def potential_relative_obstacle_calculation(obstacle, obstacle_weight, d_inf, p):
    """
    A summation of the potential in one point for all the obstacles.
      :param obstacle: x,y coordinates for one obstacle.
    :param obstacle_weight: The fixed weight of a obstacle.
    :param d_inf: The fixed zone of influence for a the obstacles.
    :param p: The x,y coordinate of the point being calculated.
    :return: The summation of the potential in one point for all the obstacles.
    """
    u_o = 0
    obstacle_shape = obstacle.shape
    for i in range(obstacle_shape[0]):
        u_o = potential_relative_obstacle_calculation_j(obstacle[i, :], obstacle_weight, d_inf, p) + u_o
    return u_o



def potential_relative_goal_calculation(goal, goal_weight, p):
    """
    The potential for one point relative the goal.
    :param goal: x,y coordinate for hte goal.
    :param goal_weight: The fixed weight of a goal.
    :param p: The x,y coordinate of the point being calculated.
    :return: The potential in one point relative the goal.
    """
    return goal_weight*np.linalg.norm(p-goal)


if __name__ == '__main__':
    p = np.array([1, 1])
    goal = np.array([4, 4])

    obstacle = np.array([[2, 2], [3, 3], [4, 4], [5, 5]])
    goal_weight = 1
    obstacle_weight = 100
    d_inf = 10
    u_o = potential_relative_obstacle_calculation(obstacle, obstacle_weight, d_inf, p)
    u_g = potential_relative_goal_calculation(goal, goal_weight, p)
    print(u_o)
    print("Potential Ug", u_g)

    # https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6607961