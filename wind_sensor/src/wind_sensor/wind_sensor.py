#!/usr/bin/env python

import sys
from subprocess import Popen, PIPE
import json
from types import SimpleNamespace

data = []
wind_sensor = Popen(['node', '../signalk-calypso-ultrasonic/test/standalone.js'], stdout=PIPE)
buffer = b''
while True:

    # read sensor data one char at the time
    out = wind_sensor.stdout.read(1)


    # after a full reading
    if out == b'\n':
        data.append(str(buffer))
        print(data)
        buffer = b''
    else:
       buffer += out

    #x = json.loads(data, object_hook=lambda d: SimpleNamespace(**d))

    #print(x.environment.wind.angleApparent)