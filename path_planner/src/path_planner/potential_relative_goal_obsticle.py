import numpy as np


def potential_relative_obstacle_calculation_j(obstacle, obstacle_weight, d_inf, p):
    if np.linalg.norm(p-obstacle) <= d_inf:
        return obstacle_weight*((1/np.linalg.norm(p-obstacle)) - (1/d_inf))
    elif np.linalg.norm(p-obstacle) > d_inf:
        return 0


def potential_relative_obstacle_calculation(obstacle, obstacle_weight, d_inf, p):
    u_o = 0
    obstacle_shape = obstacle.shape
    for i in range(obstacle_shape[0]):
        u_o = potential_relative_obstacle_calculation_j(obstacle[i, :], obstacle_weight, d_inf, p) + u_o
    return u_o
    # potential_relative_obstacle_calculation_j


def potential_relative_goal_calculation(goal, goal_weight, p):
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