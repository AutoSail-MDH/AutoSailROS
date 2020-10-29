#!/usr/bin/env python3

# Standard libs
import sys
import unittest
import math
import time
import matplotlib.pyplot as plt

# Third party libs

# Local libs
from ctrl import rudder_controller as rc
from ctrl import pid


def new_heading(pid_adjusted_heading, dt):  # Hittade bara på någonting
    """
    Used for the plot to emulate turning over time
    :param pid_adjusted_heading: heading output from the PID to compensate for drift
    :param dt: delta time
    :return: an emulated turning over time
    """
    return float(pid_adjusted_heading*dt*10.0)


def is_converged(_list, conv_point, error_margin=1e-3):
    """
    Checks the last headings to see if they have converged
    :param _list: The list of the previous headings
    :param conv_point: The point of convolution
    :param error_margin: How far from the point of convolution the heading is allowed to be
    :return: True if the past 20 values is within the error margin, else False
    """
    for i in _list[-100:]:
        list_item = abs(i - conv_point)
        if list_item > error_margin:
            return False
    return True


def run_convergence(pid, heading, _time=None, start_time=None, setpoint=None):
    """
    Run until the heading have converged
    :param pid: rudder_controller.PID class
    :param heading: list of the previous headings
    :param _time: Used for plotting. e.g. list = []
    :param start_time: Used for plotting. time.time() when the run started.
    :param setpoint: Used for plotting. e.g. list = []
    """
    last_time = time.time()
    start_time = last_time
    while not is_converged(_list=heading, conv_point=pid.setpoint):

        # Run PID and get new heading
        pid_adjusted_heading = pid(heading[-1])
        _now = time.time()
        heading += [(heading[-1] + new_heading(pid_adjusted_heading=pid_adjusted_heading, dt=_now - last_time))
                    % (2 * math.pi)]

        # Used for plotting
        if _time is not None and start_time is not None:
            _time += [_now-start_time]
        if setpoint is not None:
            setpoint += [pid.setpoint]

        # used for calculating dt in new_heading()
        last_time = _now
        if (last_time - start_time) > 60:
            return False
    return True


class TestRudder(unittest.TestCase):
    def setUp(self):
        self.pid = pid.PID(kp=2.0, ki=1, kd=0.05)

    def test_PidController_init(self):
        # PID values
        self.assertEqual(self.pid.previous_error, 0)
        self.assertEqual(self.pid.error, 0)  # Proportional term
        self.assertEqual(self.pid.integral, 0)
        self.assertEqual(self.pid.derivative, 0)

        # PID coefficients
        self.assertEqual(self.pid.kp, 2)
        self.assertEqual(self.pid.ki, 1)
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
        heading = [0.0]

        self.pid.setpoint = 3*math.pi
        converged = run_convergence(pid=self.pid, heading=heading)
        self.assertTrue(converged, "It took more than a minute to converge")

        self.pid.setpoint = 0
        converged = run_convergence(pid=self.pid, heading=heading)
        self.assertTrue(converged, "It took more than a minute to converge")

        self.pid.setpoint = 0.5 * math.pi
        converged = run_convergence(pid=self.pid, heading=heading)
        self.assertTrue(converged, "It took more than a minute to converge")

        self.pid.setpoint = 2 * math.pi
        converged = run_convergence(pid=self.pid, heading=heading)
        self.assertTrue(converged, "It took more than a minute to converge")

    def test_PID_turning(self):

        # Used for convergence check and plotting
        heading = [0.0]

        # Used for plotting
        start_time = time.time()
        setpoint = [0.0]
        _time = [0.0]

        for i in range(0, 5):
            self.pid.setpoint = 0.5*i*math.pi
            while not is_converged(heading, self.pid.setpoint):
                converged = run_convergence(self.pid, heading, _time, start_time, setpoint)
                self.assertTrue(converged, "It took more than a minute to converge")
        for i in range(6, -1, -1):
            self.pid.setpoint = 0.5*i*math.pi
            while not is_converged(heading, self.pid.setpoint):
                converged = run_convergence(self.pid, heading, _time, start_time, setpoint)
                self.assertTrue(converged, "It took more than a minute to converge")

        # Uncomment for plotting of heading
        #plt.plot(_time, heading, label='measured')
        #plt.plot(_time, setpoint, label='target')
        #plt.show()

    def test_PID_controller_switch(self):
        # Initial previous if use heading
        use_heading = True

        velocity = 1
        use_heading = rc.if_use_heading_as_setpoint(previous_bool=use_heading, velocity=velocity)
        self.assertTrue(rc.if_use_heading_as_setpoint(previous_bool=use_heading, velocity=velocity))

        velocity = 4
        use_heading = rc.if_use_heading_as_setpoint(previous_bool=use_heading, velocity=velocity)
        self.assertTrue(use_heading)

        velocity = 8
        use_heading = rc.if_use_heading_as_setpoint(previous_bool=use_heading, velocity=velocity)
        self.assertFalse(rc.if_use_heading_as_setpoint(previous_bool=use_heading, velocity=velocity))

        velocity = 4
        use_heading = rc.if_use_heading_as_setpoint(previous_bool=use_heading, velocity=velocity)
        self.assertFalse(rc.if_use_heading_as_setpoint(previous_bool=use_heading, velocity=velocity))

        velocity = 1
        use_heading = rc.if_use_heading_as_setpoint(previous_bool=use_heading, velocity=velocity)
        self.assertTrue(rc.if_use_heading_as_setpoint(previous_bool=use_heading, velocity=velocity))

    def test_rudder_angle_calculation(self):
        control_signal = 0

        self.pid.setpoint = 0
        pid_adjusted_heading = 0
        for i in range(100000):
            pid_adjusted_heading = self.pid(control_signal)
        angle = rc.calculate_rudder_angle(control_signal, pid_adjusted_heading, math.pi / 4, 2)
        self.assertAlmostEqual(angle, 0)

        self.pid.setpoint = math.pi/4
        pid_adjusted_heading = 0
        for i in range(100000):
            pid_adjusted_heading = self.pid(control_signal)
        angle = rc.calculate_rudder_angle(control_signal, pid_adjusted_heading, math.pi / 4, 2)
        self.assertAlmostEqual(angle, -math.pi/4)

        self.pid.setpoint = -math.pi / 4
        pid_adjusted_heading = 0
        for i in range(100000):
            pid_adjusted_heading = self.pid(control_signal)
        angle = rc.calculate_rudder_angle(control_signal, pid_adjusted_heading, math.pi / 4, 2)
        self.assertAlmostEqual(angle, math.pi / 4)


class Plotting:
    def __init__(self):
        self.pid = pid.PID()
        self.heading = 0
        self.pid.tunings = (3, 0.1, 0.05)

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
    import rostest
    import rosunit

    #unittest.main()
    rosunit.unitrun("ctrl_pkg", "unittest_rudder_controller", TestRudder)

