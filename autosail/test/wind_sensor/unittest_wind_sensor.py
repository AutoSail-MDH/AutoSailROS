#!/usr/bin/env python

import rosunit
from wind_sensor.wind_sensor import WindSensor
from unittest import TestCase
from unittest.mock import patch
import tempfile
import os
import math

# Global Variables, used to assert output
mock_wind_speed_val = [1.145678, 1.26456, 1.35834]
mock_wind_angle_val = [1.1042771838760873, 1.2042771838760873, 1.3042771838760873]
mock_temp = [21, 22, 23]
mock_battery = [0.1, 0.2, 0.3]
mock_roll = [0.05235987755982988, 0.05235987755982988, 0.05235987755982988]
mock_pitch = [0.5410520681182421, 0.5410520681182421, 0.5410520681182421]
mock_yaw = [1.780235837034216, 1.780235837034216, 1.780235837034216]


def write_mock_values(x):
    """
    Function which returns a mock version of the wind-sensor JSON format
    """
    if x == 0:
        return b'GOT MESSAGE: {"updates":[{"source":{"label":"Calypso Ultrasonic","type":"Ultrasonic"},' \
               b'"timestamp":"2020-12-01T12:17:22.723Z","values":[' \
               b'{"path":"environment.outside.temperature","value":21},' \
               b'{"path":"environment.wind.angleApparent","value": 1.1042771838760873},' \
               b'{"path":"environment.wind.speedApparent","value":1.145678},' \
               b'{"path":"electrical.batteries.99.name","value":"ULTRASONIC"},' \
               b'{"path":"electrical.batteries.99.location","value":"Mast"},' \
               b'{"path":"electrical.batteries.99.capacity.stateOfCharge","value":0.1},' \
               b'{"path":"navigation.attitude.roll","value":0.05235987755982988},' \
               b'{"path":"navigation.attitude.pitch","value":0.5410520681182421},' \
               b'{"path":"navigation.attitude.yaw","value":1.780235837034216},' \
               b'{"path":"navigation.headingMagnetic","value":1.780235837034216}]}]}\n'

    if x == 1:
        return b'GOT MESSAGE: {"updates":[{"source":{"label":"Calypso Ultrasonic","type":"Ultrasonic"},' \
               b'"timestamp":"2020-12-01T12:17:22.723Z","values":[' \
               b'{"path":"environment.outside.temperature","value":22},' \
               b'{"path":"environment.wind.angleApparent","value": 1.2042771838760873},' \
               b'{"path":"environment.wind.speedApparent","value":1.26456},' \
               b'{"path":"electrical.batteries.99.name","value":"ULTRASONIC"},' \
               b'{"path":"electrical.batteries.99.location","value":"Mast"},' \
               b'{"path":"electrical.batteries.99.capacity.stateOfCharge","value":0.2},' \
               b'{"path":"navigation.attitude.roll","value":0.05235987755982988},' \
               b'{"path":"navigation.attitude.pitch","value":0.5410520681182421},' \
               b'{"path":"navigation.attitude.yaw","value":1.780235837034216},' \
               b'{"path":"navigation.headingMagnetic","value":1.780235837034216}]}]}\n'

    if x == 2:
        return b'GOT MESSAGE: {"updates":[{"source":{"label":"Calypso Ultrasonic","type":"Ultrasonic"},' \
               b'"timestamp":"2020-12-01T12:17:22.723Z","values":[' \
               b'{"path":"environment.outside.temperature","value":23},' \
               b'{"path":"environment.wind.angleApparent","value": 1.3042771838760873},' \
               b'{"path":"environment.wind.speedApparent","value":1.35834},' \
               b'{"path":"electrical.batteries.99.name","value":"ULTRASONIC"},' \
               b'{"path":"electrical.batteries.99.location","value":"Mast"},' \
               b'{"path":"electrical.batteries.99.capacity.stateOfCharge","value":0.3},' \
               b'{"path":"navigation.attitude.roll","value":0.05235987755982988},' \
               b'{"path":"navigation.attitude.pitch","value":0.5410520681182421},' \
               b'{"path":"navigation.attitude.yaw","value":1.780235837034216},' \
               b'{"path":"navigation.headingMagnetic","value":1.780235837034216}]}]}\n'


class TestWindSensor(TestCase):

    def setUp(self):
        """
        Function which mocks an output to stdout by utilizing a tempor-
        ary file. Starts the wind-sensor node so its functions can be
        used.
        """
        self.patcher = patch('wind_sensor.wind_sensor.Popen')
        self.popen_mock = self.patcher.start()
        self.stdout_mock = tempfile.NamedTemporaryFile(delete=False)
        self.popen_mock.return_value.stdout = self.stdout_mock
        self.ws = WindSensor()

    def tearDown(self):
        """
        Closes the mocked stdout.
        """
        self.stdout_mock.close()
        os.remove(self.stdout_mock.name)

    def test_battery_charge(self):
        """
        Asserts a mocked battery value with the input value from stdout
        """
        x = 0
        while True:
            mock_values = write_mock_values(x)
            self.stdout_mock.write(mock_values)
            self.stdout_mock.seek(0)
            battery_charge = self.ws.get_battery_charge()
            self.assertEqual(mock_battery[x], battery_charge)
            x = x + 1
            if x > 2:
                break

    def mock_wind_vector(self, x):
        """
        Returns a mocked wind vector dependant on the input value from
        the mocked wind angle and wind speed taken from the mocked stdout.
        """
        return [math.cos(mock_wind_angle_val[x]) * mock_wind_speed_val[x],
                math.sin(mock_wind_angle_val[x]) * mock_wind_speed_val[x]]

    def test_get_rpy(self):
        """
        Asserts a mocked roll, pitch and yaw, value with the input value from stdout
        """
        x = 0
        while True:
            mock_values = write_mock_values(x)
            self.stdout_mock.write(mock_values)
            self.stdout_mock.seek(0)
            rpy_vector = self.ws.get_rpy()
            self.assertEqual(mock_roll[x], rpy_vector[0])
            self.assertEqual(mock_pitch[x], rpy_vector[1])
            self.assertEqual(mock_yaw[x], rpy_vector[2])
            x = x + 1
            if x > 2:
                break

    def test_get_tmp(self):
        """
        Asserts a mocked temperature value with the input value from stdout
        """
        x = 0
        while True:
            mock_values = write_mock_values(x)
            self.stdout_mock.write(mock_values)
            self.stdout_mock.seek(0)
            temp = self.ws.get_temp()
            self.assertEqual(mock_temp[x], temp)
            x = x + 1
            if x > 2:
                break

    def test_wind_vector(self):
        """
        Asserts a mocked wind vector value with the input value from stdout
        """
        x = 0
        while True:
            mock_values = write_mock_values(x)
            self.stdout_mock.write(mock_values)
            self.stdout_mock.seek(0)
            wind_vector = self.ws.get_wind_vector()
            self.assertEqual(self.mock_wind_vector(x), wind_vector)
            x = x + 1
            if x > 2:
                break


if __name__ == '__main__':
    rosunit.unitrun("autosail", "unittest_wind_sensor", TestWindSensor)

