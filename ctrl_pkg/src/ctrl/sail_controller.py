#!/usr/bin/env python

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


def trim_sail(sail_angle, sail_limits):
    """
    Determine the servo position given the sail angle and sail limit
    :param sail_angle: angle of the sail, in radian
    :param sail_limits: The max angle of the sail, in radian
    :return: the value of the servo, where 0 is fully released sail and 1620 is fully trimmed
    """
    # If desired value is outside of max range, return max value
    if sail_angle > sail_limits or sail_angle < -sail_limits:
        return 0

    # Calculate relation between sail angle and servo range, return proportional servo degree
    scalar = 1620 / sail_limits
    servo_degree = 1620-abs(sail_angle)*scalar
    return servo_degree
