import unittest
import sys
import math
import time
import matplotlib.pyplot as plt
sys.path.append('../scripts/')
from src.ctrl import rudder_controller as rc


def new_heading(pid_adjusted_heading, dt):  # Hittade bara på någonting
    return float(pid_adjusted_heading*dt*10)


"""
class TestPID(unittest.TestCase):
    def test_course_straight(self):
        v = 6
        heading = math.pi
        course = heading
        controller = rc.PidController
        controller.set_current_heading(heading)
        for i in range(100):
            controller()
        p = controller.
        self.assertAlmostEqual(p, 0)
        return

"""
"""
    
class Plotting:
    def __init__(self):
        self.v = 5
        self.heading = 0
        self.course = self.heading
        self.controller = rc.PidController
        self.controller.set_current_heading(self.heading)
        self.setpoint, self.y, self.x = [], [], []
        self.rudder_angle = 0
    def plot(self):
        start_time = time.time()
        last_time = start_time
        while time.time() - start_time < 10:
            current_time = time.time()
            dt = current_time - last_time
            rudder_angle = rc.rudder_angle_calculation(self.heading, )
            self.heading += angular_vel(rudder_angle=rudder_angle, velocity=self.v)*dt
            power = pid(water_temp)
            heading =

            self.x += [current_time - start_time]
            self.y += [heading]
            self.setpoint += [controller.pid.setpoint]

            if current_time - start_time > 1:
                pid.setpoint = 100

            last_time = current_time
"""


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
    plot = Plotting()
    plot.plot()
