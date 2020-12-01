#!/usr/bin/env python

import rosunit
from wind_sensor.wind_sensor import WindSensor
from unittest import TestCase
from unittest.mock import patch
import tempfile
import os
import math

mock_temp = 299.15
mock_battery = 0.2
mock_roll = 0.05235987755982988
mock_pitch = 0.5410520681182421
mock_yaw = 1.780235837034216
wind_speed_val = 0.001
wind_angle_val = 1.2042771838760873

mock_wind_vector = [math.cos(wind_angle_val)*wind_speed_val, math.sin(wind_angle_val)*wind_speed_val]

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
        self.assertEqual(mock_battery, battery_charge)

    def test_get_rpy(self):
        mock_values = write_mock_values()
        self.stdout_mock.write(mock_values)
        self.stdout_mock.seek(0)
        rpy_vector = self.ws.get_rpy()
        self.assertEqual(mock_roll, rpy_vector[0])
        self.assertEqual(mock_pitch, rpy_vector[1])
        self.assertEqual(mock_yaw, rpy_vector[2])

    def test_get_tmp(self):
        mock_values = write_mock_values()
        self.stdout_mock.write(mock_values)
        self.stdout_mock.seek(0)
        temp = self.ws.get_temp()
        self.assertEqual(mock_temp, temp)

    def test_wind_vector(self):
        mock_values = write_mock_values()
        self.stdout_mock.write(mock_values)
        self.stdout_mock.seek(0)
        wind_vector = self.ws.get_wind_vector()
        self.assertEqual(mock_wind_vector, wind_vector)


if __name__ == '__main__':
    rosunit.unitrun("autosail", "unittest_wind_sensor", TestWindSensor)




# def write_mock_values():
#     return b'GOT MESSAGE: {"updates":[{"source":{"label":"Calypso Ultrasonic","type":"Ultrasonic"},"timestamp":"2020-12-01T12:17:22.723Z","values":[{"path":"environment.outside.temperature","value":299.15},{"path":"environment.wind.angleApparent","value":0},{"path":"environment.wind.speedApparent","value":0},{"path":"electrical.batteries.99.name","value":"ULTRASONIC"},{"path":"electrical.batteries.99.location","value":"Mast"},{"path":"electrical.batteries.99.capacity.stateOfCharge","value":0.1}]}]}\n'
#
#
# class TestWindSensor(TestCase):
#
#     def setUp(self):
#         self.ws = WindSensor()
#
#     @patch('wind_sensor.wind_sensor.Popen')
#     def test_stdout(self, mock_stdout):
#         mock_stdout.read_line.return_value = write_mock_values()
#         battery_charge = self.ws.get_battery_charge()
#         print(battery_charge)
#         self.assertEquals(0.1, battery_charge)
#
#
# if __name__ == '__main__':
#     rosunit.unitrun("autosail", "unittest_wind_sensor", TestWindSensor)