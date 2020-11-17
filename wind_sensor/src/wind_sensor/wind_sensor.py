#!/usr/bin/env python

from subprocess import Popen, PIPE
import json

data = []
wind_sensor = Popen(['node', '../signalk-calypso-ultrasonic/test/standalone.js'], stdout=PIPE)
buffer = b''
while True:

    # read sensor data one char at the time
    out = wind_sensor.stdout.read(1)


    # after a full reading
    if out == b'\n':
        #data.append(str(buffer))
        x = json.loads(buffer.decode("utf-8"))
        print(x["updates"][0]["values"][0]["path"])
        buffer = b''
    else:
        buffer += out






