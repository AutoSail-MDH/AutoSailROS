#!/usr/bin/env python3
# Standard libs
import math


def rudder_angle_calculation(current_heading, pid_corrected_heading, rudder_limit, velocity):
    """
    Calculates the angle of the rudder given
    :param current_heading:
    :param pid_corrected_heading:
    :param rudder_limit:
    :param velocity:
    :return:
    """
    if math.cos(current_heading-pid_corrected_heading) < 0:
        return math.copysign(1, velocity)*math.copysign(rudder_limit, math.sin(current_heading-pid_corrected_heading))
    else:
        return math.copysign(math.sin(current_heading-pid_corrected_heading)*rudder_limit, velocity)


def trajectory_to_relative_heading(desired_trajectory, current_course):
    """
    Transforms the course to a heading
    :param desired_trajectory: The desired direction in global frame
    :param current_course: The current direction in global frame
    :return: The heading of the vessel
    """
    heading = desired_trajectory-current_course
    return heading


def if_use_heading_as_setpoint(previous_bool, velocity=0, upper_threshold=5, lower_threshold=3):
    """
    Return True if heading should be used as the setpoint for the PID, depending on the velocity of the vessel and
    latching with an upper and lower threshold.
    :param velocity: The velocity of the vessel
    :param previous_bool: True if heading is currently used as setpoint
    :param upper_threshold: Threshold for switching to course controller
    :param lower_threshold: Threshold for switching to heading controller
    :return: True if heading controller should be used, False if course controller should be used
    """
    if velocity < lower_threshold:
        return True
    elif velocity > upper_threshold:
        return False
    else:
        return previous_bool




