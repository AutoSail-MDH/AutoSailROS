#!/usr/bin/env python

from subprocess import Popen, PIPE
import json
import time

data = []
wind_sensor = Popen(['node', '../signalk-calypso-ultrasonic/test/standalone.js'], stdout=PIPE)
buffer = b''

# wait 5 seconds to fill index
time.sleep(3)

while True:

    # read sensor data one char at the time
    out = wind_sensor.stdout.read(1)

    # read to EOL of stdout
    if out == b'\n':
        x = json.loads(buffer.decode("utf-8"))

        # EnvOutTmp
        print(x["updates"][0]["values"][0]["path"], " :\t",
              x["updates"][0]["values"][0]["value"])

        # EnvWindAngleApp
        print(x["updates"][0]["values"][1]["path"], "  :\t",
              x["updates"][0]["values"][1]["value"])

        # EnvWindSpeedApp
        print(x["updates"][0]["values"][2]["path"], "  :\t",
              x["updates"][0]["values"][2]["value"])

        # ElecBattName
        print(x["updates"][0]["values"][3]["path"], "  :\t",
              x["updates"][0]["values"][3]["value"])

        # ElecBattLoc
        print(x["updates"][0]["values"][4]["path"], "  :\t",
              x["updates"][0]["values"][4]["value"])

        # ElecBattCapacity
        print(x["updates"][0]["values"][5]["path"], "  :\t",
              x["updates"][0]["values"][5]["value"])

        # # NavAttRoll
        # print(x["updates"][0]["values"][6]["path"], "  :\t",
        #       x["updates"][0]["values"][6]["value"])

        # # NavAttPitch
        # print(x["updates"][0]["values"][7]["path"], "  :\t",
        #       x["updates"][0]["values"][7]["value"])
        #
        # # NavAttYaw
        # print(x["updates"][0]["values"][8]["path"], "  :\t",
        #       x["updates"][0]["values"][8]["value"])
        #
        # # NavAttHeadingMagnetic
        # print(x["updates"][0]["values"][9]["path"], "  :\t",
        #       x["updates"][0]["values"][9]["value"])

        buffer = b''

    else:
        buffer += out






