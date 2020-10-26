#!/usr/bin/env python3
# Standard libs
import math
import time


def rudder_angle_calculation(current_heading, pid_corrected_heading, rudder_limit, velocity):
    if math.cos(current_heading-pid_corrected_heading) < 0:
        return math.copysign(1, velocity)*math.copysign(rudder_limit, math.sin(current_heading-pid_corrected_heading))
    else:
        return math.copysign(math.sin(current_heading-pid_corrected_heading)*rudder_limit, velocity)


def trajectory_to_relative_heading(desired_trajectory, current_heading):
    heading = desired_trajectory-current_heading
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


class PID:
    def __init__(self, kp=1.0, ki=0.1, kd=0.05, setpoint=0, output_limits=(-math.pi, math.pi), sample_time=0.01):
        self.previous_error = 0
        self.integral = 0
        self.derivative = 0
        self.error = 0

        # Check if setpoint is in the first revolution, else make it so
        if 0 <= setpoint < 2 * math.pi:
            self._setpoint = setpoint
        else:
            self._setpoint = math.atan2(math.sin(setpoint), math.cos(setpoint))

        # The coefficients of the PID
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.sample_time = sample_time
        self.min_value = output_limits[0]
        self.max_value = output_limits[1]

        if self.min_value > self.max_value:
            raise ValueError("min must be greater than max")

        self.last_time = time.time()
        self.last_output = None
        self.last_control_signal = 0

    def __call__(self, control_signal):
        # Computes the time since last call
        _now = time.time()
        dt = _now-self.last_time if _now-self.last_time else 1e-15

        # Returns last value if the PID have recently ran
        if dt < self.sample_time and self.last_output is not None:
            return self.last_output

        # The PID value calculations
        self.error = math.atan2(math.sin(self._setpoint-control_signal), math.cos(self._setpoint-control_signal))
        self.integral += self.error*dt
        self.integral = max(min(self.integral, self.max_value), self.min_value)  # Clamed to avoid integral windup
        self.derivative = (self.error-self.previous_error)/dt

        # Processing of the output
        self.previous_error = self.error
        output = self.kp*self.error+self.ki*self.integral+self.kd*self.derivative
        output = max(min(output, self.max_value), self.min_value)  # Clamed according to the output restrictions

        # Saves status of the PID
        self.last_output = output
        self.last_control_signal = control_signal
        self.last_time = _now

        return output

    @property
    def setpoint(self):
        return self._setpoint

    @setpoint.setter
    def setpoint(self, new_setpoint):
        if not (0 <= new_setpoint < 2*math.pi):
            self._setpoint = math.atan2(math.sin(new_setpoint), math.cos(new_setpoint))
        else:
            self._setpoint = new_setpoint
        self.reset()

    def set_limits(self, limits):
        if limits(0) > limits(1):
            raise ValueError("min must be greater than max")
        self.min_value = limits(0)
        self.max_value = limits(1)

    def reset(self):
        self.error = 0
        self.integral = 0
        self.derivative = 0

        self.last_time = time.time()
        self.last_output = 0
        self.last_control_signal = 0

