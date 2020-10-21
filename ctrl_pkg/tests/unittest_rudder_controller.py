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
    return float(pid_adjusted_heading*dt*10)


class TestPID(unittest.TestCase):
    def test_PidController_init(self):
        self.pid = rc.PidController(kp=1, ki=2, kd=3, limits=math.pi/4)
        self.assertEqual(self.pid.current_heading, None)
        self.assertEqual(self.pid.desired_course, None)
        self.assertEqual(self.pid.desired_heading, None)
        self.assertEqual(self.pid.heading_flag, True)
        self.assertEqual(self.pid.pid.tunings, (1, 2, 3))
        self.assertEqual(self.pid.pid.output_limits, (-math.pi/4, math.pi/4))

    def test_set_limits(self):
        self.pid.set_limits((-1, 1))
        self.assertEqual(self.pid.pid.output_limits, (-1, 1))
        self.pid.set_limits((-2, 2))
        self.assertEqual(self.pid.pid.output_limits, (-2, 2))

    def test_use_heading(self):
        self.pid.use_heading(False)
        self.assertEqual(self.pid.heading_flag, False)
        self.pid.use_heading(True)
        self.assertEqual(self.pid.heading_flag, True)

    def test_update_setpoint(self):
        self.pid.update_setpoint(2)
        self.assertEqual(self.pid.pid.setpoint, 2)
        self.pid.update_setpoint(100)
        self.assertEqual(self.pid.pid.setpoint, 100)
        self.pid.update_setpoint(0)
        self.assertEqual(self.pid.pid.setpoint, 0)
        self.pid.update_setpoint(-1)
        self.assertEqual(self.pid.pid.setpoint, -1)

    def test_set_current_heading(self):
        self.pid.set_current_heading(-2)
        self.assertEqual(self.pid.current_heading, -2)
        self.pid.set_current_heading(0)
        self.assertEqual(self.pid.current_heading, 0)
        self.pid.set_current_heading(1)
        self.assertEqual(self.pid.current_heading, 1)

    def test_setpoint_to_use(self):
        self.pid.heading_flag = True
        self.assertEqual(self.pid.setpoint_to_use(), self.pid.desired_heading)
        self.pid.heading_flag = False
        self.assertEqual(self.pid.setpoint_to_use(), self.pid.desired_course)


class Plotting:
    def __init__(self):
        self.pid = rc.PidController().pid
        self.heading = 0
        self.pid.tunings = (1, 0.001, 0.005)

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

            if current_time - start_time > 1:
                self.pid.setpoint = math.pi*2

            last_time = current_time

        plt.plot(x, y, label='measured')
        plt.plot(x, setpoint, label='target')
        plt.xlabel('time')
        plt.ylabel('temperature')
        plt.legend()
        plt.show()


if __name__ == "__main__":
    # Plot the PID
    # plot = Plotting()
    # plot.plot()
    test = TestPID()
    test.test_PidController_init()
    test.test_set_limits()
    test.test_use_heading()
    test.test_update_setpoint()
    test.test_set_current_heading()
    test.test_setpoint_to_use()

