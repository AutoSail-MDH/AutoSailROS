import sys
import rospy
import math
from simple_pid import PID




def rudder_angle_calculation(pid_corrected_heading, velocity):
    return rudder_angle


def pid_(desired_heading, course, heading, velocity_flag=False):
    """
    :param course: The current course
    :param heading: The current heading
    :param desired_heading: The desired heading of the vessel, in degrees
    :param velocity_flag: True if the velocity is above the threshold
    :return: The new angle of the rudder
    """

    kp = 1  # The Proportional term scalar
    ki = 1  # The Integral term scalar
    kd = 1  # The Derivative term scalar
    pid = PID(kp, ki, kd, 0)  # Initialization of the PID, with 0 as the control signal
    pid.output_limits = (-45, 45)
    while True:
        if velocity_flag:
            control_signal = course
        else:
            control_signal = heading
        control = pid(control_signal)

def trajectory_to_relative_heading(desired_trajectory, current_heading):
    heading = desired_trajectory-current_heading
    return heading


if __name__ == "__main__":
    wait_for_service('path_planner')  # TODO: change to the correct name of the path planner topic