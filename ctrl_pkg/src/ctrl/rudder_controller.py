import math
from simple_pid import PID


def rudder_angle_calculation(current_heading, pid_corrected_heading, rudder_limit, velocity):
    if math.cos(current_heading-pid_corrected_heading) < 0:
        return math.copysign(1, velocity)*math.copysign(rudder_limit, math.sin(current_heading-pid_corrected_heading))
    else:
        return math.copysign(math.sin(current_heading-pid_corrected_heading)*rudder_limit, velocity)


def trajectory_to_relative_heading(desired_trajectory, current_heading):
    heading = desired_trajectory-current_heading
    return heading


def use_heading_as_setpoint(previous_bool, velocity=0, upper_threshold=5, lower_threshold=3):
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


class PidController:
    def __init__(self, kp=1, ki=0.1, kd=0.05, limits=math.pi/4):
        """
        Initialization of the PID controller
        :param kp: Proportional coefficient of the PID
        :param ki: Integral coefficient of the PID
        :param kd: Derivative coefficient of the PID
        :param limits: The limits of the output and integral term
        """
        self.current_heading = None
        self.desired_course = None
        self.desired_heading = None
        self.heading_flag = True
        self.pid = PID(kp, ki, kd, 0)  # Initialization of the PID, with 0 as the control signal
        self.pid.output_limits = (-limits, limits)  # Limits the rudder angles to [-45, 45] degrees

    def set_limits(self, limits):
        """
        Update the limits of the rudder, in radian
        :param limits: [lower limit, upper limit], e.g. [-pi/2, pi/2]
        :return:
        """
        self.pid.output_limits = (limits[0], limits[1])

    def use_heading(self, flag):
        """
        Used to switch between course and heading control
        :param flag: True for heading, False for course
        :return:
        """
        self.heading_flag = flag

    def update_setpoint(self, setpoint):
        """
        Change the setpoint of the PID, e.g. to a new desired heading
        :param setpoint: The new setpoint
        :return:
        """
        self.pid.setpoint = setpoint

    def set_current_heading(self, heading):
        """
        Update the current heading
        :param heading: The new heading
        :return:
        """
        self.current_heading = heading

    def setpoint_to_use(self):
        """
        Evaluates if heading or course controller should be used depending on the heading_flag
        :return:
        """
        if self.heading_flag:
            return self.desired_heading
        else:
            return self.desired_course

    def __call__(self):
        return self.pid(self.current_heading)


