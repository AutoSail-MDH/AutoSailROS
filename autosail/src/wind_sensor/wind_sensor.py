#!/usr/bin/env python

import json
from subprocess import Popen, PIPE
import sys
import os
import collections
import time
import threading
import math


class WindSensor:
    def __init__(self):
        self.wind_sensor = Popen(['node', os.path.dirname(sys.argv[0]) + '/../../src/signalk-calypso-ultrasonic/test/standalone.js'], stdout=PIPE)
        self.data = {}
        self.q = collections.deque(maxlen=1)
        self.readThread = threading.Thread(target=self.read_output)
        self.readThread.daemon = True
        self.readThread.start()

    def read_output(self):
        for line in iter(self.wind_sensor.stdout.readline, ""):
            self.q.append(line)

    def update(self):
        line = b''
        while not line:
            line = b''.join(self.q)
        self.data = json.loads(line.decode("utf-8")[13:])
        #print(line) # use this to view the whole line

    def get_data(self, path):
        for val in self.data["updates"][0]["values"]:
            if val['path'] == path:
                return val['value']
        return 0

    def get_wind_vector(self):
        self.update()
        wind_angle_val = float(self.get_data("environment.wind.angleApparent"))
        wind_speed_val = float(self.get_data("environment.wind.speedApparent"))
        if wind_speed_val == 0:
            wind_speed_val = 0.001

        wind_vector = [math.cos(wind_angle_val)*wind_speed_val, math.sin(wind_angle_val)*wind_speed_val]
        return wind_vector

    def get_rpy(self):
        self.update()
        return [
            float(self.get_data("navigation.attitude.roll")),
            float(self.get_data("navigation.attitude.pitch")),
            float(self.get_data("navigation.attitude.yaw"))
        ]

    def get_temp(self):
        self.update()
        return float(self.get_data("environment.outside.temperature"))

    def get_battery_charge(self):
        self.update()
        return float(self.get_data("electrical.batteries.99.capacity.stateOfCharge"))

    def get_values(self):
        self.update()
        return self.data["updates"][0]["values"]

    def __del__(self):
        self.readThread.join()
        self.wind_sensor.kill()


if __name__ == "__main__":
    ws = WindSensor()
    ws.update()
