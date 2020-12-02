#!/usr/bin/env python

import rosunit
from wind_sensor.wind_sensor import WindSensor
from unittest import TestCase
from unittest.mock import patch
import tempfile
import os
import math


mock_wind_speed_val = [0.001, 0.001, 0.001]
mock_wind_angle_val = [1.2042771838760873, 1.2042771838760873, 1.2042771838760873]
mock_temp = [299.15, 299.15, 299.15]
mock_battery = [0.2, 0.2, 0.2]
mock_roll = [0.05235987755982988, 0.05235987755982988, 0.05235987755982988]
mock_pitch = [0.5410520681182421, 0.5410520681182421, 0.5410520681182421]
mock_yaw = [1.780235837034216, 1.780235837034216, 1.780235837034216]

x = 0


mock_wind_vector = [math.cos(mock_wind_angle_val[x]) * float(mock_wind_speed_val[x]),
                    math.sin(mock_wind_angle_val[x]) * float(mock_wind_speed_val[x])]


def write_mock_values():

    return b'GOT MESSAGE: {"updates":[{"source":{"label":"Calypso Ultrasonic","type":"Ultrasonic"},' \
           b'"timestamp":"2020-12-01T12:17:22.723Z","values":[' \
           b'{"path":"environment.outside.temperature","value":299.15},' \
           b'{"path":"environment.wind.angleApparent","value": 1.2042771838760873},' \
           b'{"path":"environment.wind.speedApparent","value":0},' \
           b'{"path":"electrical.batteries.99.name","value":"ULTRASONIC"},' \
           b'{"path":"electrical.batteries.99.location","value":"Mast"},' \
           b'{"path":"electrical.batteries.99.capacity.stateOfCharge","value":0.2},' \
           b'{"path":"navigation.attitude.roll","value":0.05235987755982988},' \
           b'{"path":"navigation.attitude.pitch","value":0.5410520681182421},' \
           b'{"path":"navigation.attitude.yaw","value":1.780235837034216},' \
           b'{"path":"navigation.headingMagnetic","value":1.780235837034216}]}]}\n'


class TestWindSensor(TestCase):

    def setUp(self):
        self.patcher = patch('wind_sensor.wind_sensor.Popen')
        self.popen_mock = self.patcher.start()
        self.stdout_mock = tempfile.NamedTemporaryFile(delete=False)
        self.popen_mock.return_value.stdout = self.stdout_mock
        self.ws = WindSensor()

    def tearDown(self):
        self.stdout_mock.close()
        os.remove(self.stdout_mock.name)

    def test_battery_charge(self):
        mock_values = write_mock_values()
        self.stdout_mock.write(mock_values)
        self.stdout_mock.seek(0)
        battery_charge = self.ws.get_battery_charge()
        self.assertEqual(mock_battery[x], battery_charge)

    def test_get_rpy(self):
        mock_values = write_mock_values()
        self.stdout_mock.write(mock_values)
        self.stdout_mock.seek(0)
        rpy_vector = self.ws.get_rpy()
        self.assertEqual(mock_roll[x], rpy_vector[0])
        self.assertEqual(mock_pitch[x], rpy_vector[1])
        self.assertEqual(mock_yaw[x], rpy_vector[2])

    def test_get_tmp(self):
        mock_values = write_mock_values()
        self.stdout_mock.write(mock_values)
        self.stdout_mock.seek(0)
        temp = self.ws.get_temp()
        self.assertEqual(mock_temp[x], temp)

    def test_wind_vector(self):
        mock_values = write_mock_values()
        self.stdout_mock.write(mock_values)
        self.stdout_mock.seek(0)
        wind_vector = self.ws.get_wind_vector()
        self.assertEqual(mock_wind_vector, wind_vector)


if __name__ == '__main__':
    rosunit.unitrun("autosail", "unittest_wind_sensor", TestWindSensor)


