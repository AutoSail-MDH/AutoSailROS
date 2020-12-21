#!/usr/bin/env python

import json
from subprocess import Popen, PIPE
import sys
import os
import collections
import threading
import math


class WindSensor:
    def __init__(self):
        """
        Opens up communication to the wind sensor through a javascript and starts a thread to
        continously read the data published.
        """
        self.node_proc = Popen(['node', os.path.dirname(sys.argv[0]) +
                                '/../../src/signalk-calypso-ultrasonic/test/standalone.js'], stdout=PIPE)
        self.data = {}
        self.q = collections.deque(maxlen=1)
        self.stop_thread = False
        self.readThread = threading.Thread(target=self.read_output)
        self.readThread.daemon = True
        self.readThread.start()

    def read_output(self):
        """
        Reads the output line by line and adds it to the line variable.
        """
        while not self.stop_thread:
            for line in iter(self.node_proc.stdout.readline, ""):
                if self.stop_thread:
                    break
                self.q.append(line)

    def update(self):
        """
        Updates data with the last line read and decodes it to a string.
        """
        line = b''
        while not line:
            line = b''.join(self.q)
        self.data = json.loads(line.decode("utf-8")[13:])
        #print(line) # use this to view the whole line

    def get_data(self, path):
        """
        Returns the value for the given path. For example a path such as "environment.wind.angleApparent"
        returns the value of the winds apparent angle from the wind sensor.
        :param path: The desired path to the value
        :type path: String
        :return: Returns 0 when called but the connection is not fully established. Returns a float with the requested
        value oterwise.
        :rtype: Integer when 0. Otherwise Float64.
        """
        for val in self.data["updates"][0]["values"]:
            if val['path'] == path:
                return val['value']
        return 0

    def get_wind_vector(self):
        """
        Returns the values for the wind vector based on the apparent wind angle and speed measured from the wind
        sensor.
        :return: returns the wind vector [x,y].
        :rtype: Float64 list.
        """
        self.update()
        wind_angle_val = float(self.get_data("environment.wind.angleApparent"))
        wind_speed_val = float(self.get_data("environment.wind.speedApparent"))
        if wind_speed_val == 0:
            wind_speed_val = 0.001

        wind_vector = [math.cos(wind_angle_val)*wind_speed_val, math.sin(wind_angle_val)*wind_speed_val]
        return wind_vector

    def get_rpy(self):
        """
        Returns the values of the roll, pitch, and yaw of the wind sensor.
        :return: returns the rpy list containing [roll(x), pitch(y), yaw(z)].
        :rtype: Float64 list.
        """
        self.update()
        return [
            float(self.get_data("navigation.attitude.roll")),
            float(self.get_data("navigation.attitude.pitch")),
            float(self.get_data("navigation.attitude.yaw"))
        ]

    def get_temp(self):
        """
        Returns the current temperature of the wind sensor.
        :return: returns the current measured temperature in kelvin.
        :rtype: Float64
        """
        self.update()
        return float(self.get_data("environment.outside.temperature"))

    def get_battery_charge(self):
        """
        Returns the measured battery charge in decimals. 0,1 , 0,2 , 0,3 etc. Seems like 0,1 = 10% and so
        on.
        :return: returns the value of the battery charge level.
        :rtype: Float64
        """
        self.update()
        return float(self.get_data("electrical.batteries.99.capacity.stateOfCharge"))

    def close(self):
        """
        Closes the thread and kills the connection to bluetooth through the javascript.
        """
        self.stop_thread = True
        self.readThread.join()
        self.node_proc.kill()

    def __del__(self):
        self.close()


if __name__ == "__main__":
    ws = WindSensor()
    ws.update()