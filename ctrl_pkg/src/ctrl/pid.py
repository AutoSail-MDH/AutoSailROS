import math
import time


class PID:
    def __init__(self, kp=1.0, ki=0.1, kd=0.05, setpoint=0, output_limits=(-math.pi, math.pi), sample_time=0.01):
        self.previous_error = 0
        self.integral = 0
        self.derivative = 0
        self.error = 0

        # Check if setpoint is in the first revolution, else make it so
        if -math.pi <= setpoint < math.pi:
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
        if not (-math.pi <= new_setpoint < math.pi):
            self._setpoint = math.atan2(math.sin(new_setpoint), math.cos(new_setpoint))
        else:
            self._setpoint = new_setpoint
        self.reset()

    def set_limits(self, limits):
        if limits[0] > limits[1]:
            raise ValueError("min must be greater than max")
        self.min_value = limits[0]
        self.max_value = limits[1]

    def reset(self):
        self.error = 0
        self.integral = 0
        self.derivative = 0

        self.last_time = time.time()
        self.last_output = None
        self.last_control_signal = 0