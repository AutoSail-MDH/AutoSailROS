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
        self.readThread = threading.Thread(target=self.read_output, args=(self, q.append))
        self.readThread.daemon = True
        self.readThread.start()

    def read_output(self, append):
        for line in iter(self.wind_sensor.stdout.readline, ""):
            append(line)

    def update(self):
        line = b''.join(self.q)
        self.data = json.loads(line.decode("utf-8")[13:])

    def get_data(self, path):
        for val in self.data["updates"][0]["values"]:
            if val['path'] == path:
                return val['value']
        return None

    def __del__(self):
        self.readThread.join()


if __name__ == "__main__":
    ws = WindSensor()
    ws.read_sensor()
