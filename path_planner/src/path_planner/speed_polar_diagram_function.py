#!/usr/bin/env python
import numpy as np
import rospy
import std_msgs.msg


def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]

def speed_polar_diagram_calculation(w_speed, w_theta):

    up_beat = np.array([45, 42.4, 40.5, 37.5, 35.4, 34.1, 33.5, 32.8, 32.6])
    dn_beat = np.array([139.4, 143.1, 146.4, 152.4, 163.7, 169.2, 170.8, 169.7, 148.1])
    angles = np.array([32, 36, 40, 45, 60, 70, 80, 90, 100, 110, 120, 130, 135, 140, 150, 160, 170, 180])
    speed_array = np.array([4, 6, 8, 10, 12, 14, 16, 20, 25])
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
    if w_speed < 4 : w_speed = 4  # wind speed < 4 set to 4
    if w_speed > 25: w_speed = 25  # winds peed > 25 set to 25
    w_speed = find_nearest(speed_array, w_speed)  # round wind speed to closest hole number 4,6,8,10,12,14,16,20,25
    w_speed_index = np.where(speed_array == w_speed)  # remap the wind speed to the corresponding index in the arrays.
    w_speed_index = w_speed_index[0]

    high_speed_index = np.where(speed_diagram == max(np.amax(speed_diagram[:, w_speed_index], axis=1)))
    rel_angle = angles[high_speed_index[0][0]] + w_theta
    # these functions finds the angle to max_vel it is not needed

    max_vel = max(np.amax(speed_diagram[:, w_speed_index], axis=1))

    return [max_vel, max(up_beat[w_speed_index]), max(dn_beat[w_speed_index]), w_theta]

if __name__ == '__main__':
    w_speed = 4
    w_theta = 20
    [max_vel, up_beat, dn_beat, w_theta] = speed_polar_diagram_calculation(w_speed, w_theta)
    print(max_vel)
    print(up_beat)
    print(dn_beat)

#https://www.seapilot.com/wp-content/uploads/2018/05/A35.txt