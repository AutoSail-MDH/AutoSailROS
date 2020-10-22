import unittest
import sys
import math
import time
import matplotlib.pyplot as plt
sys.path.append('../scripts/')
from src.ctrl import rudder_controller as rc


def new_heading(pid_adjusted_heading, dt):  # Hittade bara på någonting
    """
    Used for the plot to emulate turning over time
    :param pid_adjusted_heading: heading output from the PID to compensate for drift
    :param dt: delta time
    :return: an emulated turning over time
    """
    return float(pid_adjusted_heading*dt*10.0)


def is_converging(_list, conv_point, error_margin=1e-3):
    for i in _list[-10:]:
        list_item = abs(i - conv_point)
        if list_item > error_margin:
            return False
    return True


class TestPID(unittest.TestCase):
    def setUp(self):
        self.pid = rc.PID(kp=2.0, ki=0.1, kd=0.05)

    def test_PidController_init(self):
        # PID values
        self.assertEqual(self.pid.previous_error, 0)
        self.assertEqual(self.pid.error, 0)  # Proportional term
        self.assertEqual(self.pid.integral, 0)
        self.assertEqual(self.pid.derivative, 0)

        # PID coefficients
        self.assertEqual(self.pid.kp, 2)
        self.assertEqual(self.pid.ki, 0.1)
        self.assertEqual(self.pid.kd, 0.05)

        # PID settings
        self.assertEqual(self.pid.setpoint, 0)
        self.assertEqual(self.pid.sample_time, 0.01)
        self.assertEqual(self.pid.min_value, -math.pi)
        self.assertEqual(self.pid.max_value, math.pi)
        self.assertEqual(self.pid.last_output, None)
        self.assertEqual(self.pid.last_control_signal, 0)

    def test_PID_convergence(self):
        last_time = time.time()
        check_time = last_time
        heading = [0.0]
        self.pid.setpoint = math.pi
        while True:
            _now = time.time()
            pid_adjusted_heading = self.pid(heading[-1])
            heading += [heading[-1]+new_heading(pid_adjusted_heading=pid_adjusted_heading, dt=_now-last_time)]
            if abs(_now-check_time) > 0.5:
                check_time = _now
                if is_converging(_list=heading, conv_point=self.pid.setpoint, error_margin=1e-2):
                    break
            last_time = _now

    def test_PID_turning(self):
        start_time = time.time()
        self.pid.setpoint = 0


class Plotting:
    def __init__(self):
        self.pid = rc.PID()
        self.heading = 0
        self.pid.tunings = (2, 0.001, 0.005)

    def plot(self):
        start_time = time.time()
        last_time = start_time
        # keep track of values for plotting
        setpoint, y, x = [], [], []
        while time.time() - start_time < 10:
            current_time = time.time()
            dt = current_time - last_time

            pid_adjusted_heading = self.pid(self.heading)
            self.heading += new_heading(pid_adjusted_heading=pid_adjusted_heading, dt=dt)

            x += [current_time - start_time]
            y += [self.heading]
            setpoint += [self.pid.setpoint]

            if (current_time - start_time > 1) & (current_time - start_time < 6):
                self.pid.setpoint = math.pi
            elif current_time - start_time > 6:
                self.pid.setpoint = 2*math.pi-0.1
            last_time = current_time

        plt.plot(x, y, label='measured')
        plt.plot(x, setpoint, label='target')
        plt.xlabel('time')
        plt.ylabel('heading')
        plt.legend()
        plt.show()


if __name__ == "__main__":
    # Plot the PID
    # plot = Plotting()
    # plot.plot()
    unittest.main()
