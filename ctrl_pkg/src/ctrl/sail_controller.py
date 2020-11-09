#!/usr/bin/env python3
import numpy as np

sail_angle_rad = 0


def calculate_sail_angle(current_pid_roll, max_roll, max_sail):
    """
    Calculating the optimum angle the mast should be, dependant on the roll of the vessel
    :param current_pid_roll: The desired roll change, as given by the PID, in radians
    :param max_roll: The maximum allowed roll, in radians
    :param max_sail: The angle between the sail and the central axis of the vessel in radians when the sail is released
    :return: The desired angle of the sail in radians
    """
    if current_pid_roll >= max_roll:
        return max_sail
    elif current_pid_roll <= 0:
        return 0

    if max_roll == 0:
        raise ValueError("max roll can not be 0")
    return current_pid_roll / max_roll * max_sail


def trim_sail(sail_angle, sail_limits, scalar):
    """
    Using the sail angle to adjust the servo which is trimming the sail
    :param sail_angle: angle of the sail
    :return: the amount of degree the servo should be positioned in 1620 loos sail and 0 beeing maximum trim
    """
    if sail_angle > sail_limits:
        return 0
    elif sail_angle < 0:
        return sail_limits * scalar
    servo_degree = np.multiply(sail_limits-abs(sail_angle), scalar)
    return np.abs(servo_degree)
