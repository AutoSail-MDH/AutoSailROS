#!/usr/bin/env python

import json
from subprocess import Popen, PIPE
import sys
import os


class WindSensor:

    def __init__(self):
        self.wind_sensor = Popen(['node', os.path.dirname(sys.argv[0]) + '/../signalk-calypso-ultrasonic/test/standalone.js'], stdout=PIPE)
        self.buffer = b''

    def read_sensor(self):

        while True:

            # read sensor data one char at the time
            out = self.wind_sensor.stdout.read(1)

            # read to EOL of stdout
            if out == b'\n':

                x = json.loads(self.buffer.decode("utf-8")[13:])

                try:
                    # EnvironmentOutsideTemperature
                    out_temp_path = x["updates"][0]["values"][0]["path"]
                    out_temp_val = x["updates"][0]["values"][0]["value"]
                    print(out_temp_path, " :\t", out_temp_val)
                except IndexError:
                    out_temp_path = ''
                    out_temp_val = ''

                try:
                    # EnvironmentWindAngleApparent
                    wind_angle_path = x["updates"][0]["values"][1]["path"]
                    wind_angle_val = x["updates"][0]["values"][1]["value"]
                    print(wind_angle_path, "  :\t", wind_angle_val)
                except IndexError:
                    wind_angle_path = ''
                    wind_angle_val = ''

                try:
                    # EnvironmentWindSpeedApparent
                    wind_speed_path = x["updates"][0]["values"][2]["path"]
                    wind_speed_val = x["updates"][0]["values"][2]["value"]
                    print(wind_speed_path, "  :\t", wind_speed_val)
                except IndexError:
                    wind_speed_path = ''
                    wind_speed_val = ''

                try:
                    # ElectricityBatteryName
                    battery_name_path = x["updates"][0]["values"][3]["path"]
                    battery_name_val = x["updates"][0]["values"][3]["value"]
                    print(battery_name_path, "  :\t", battery_name_val)
                except IndexError:
                    battery_name_path = ''
                    battery_name_val = ''

                try:
                    # ElectricityBatteryLoc
                    battery_loc_path = x["updates"][0]["values"][4]["path"]
                    battery_loc_val = x["updates"][0]["values"][4]["value"]
                    print(battery_loc_path, "  :\t", battery_loc_val)
                except IndexError:
                    battery_loc_path = ''
                    battery_loc_val = ''

                try:
                    # ElectricityBatteryCapacity
                    battery_cap_path = x["updates"][0]["values"][5]["path"]
                    battery_cap_val = x["updates"][0]["values"][5]["value"]
                    print(battery_cap_path, "  :\t", battery_cap_val)

                except IndexError:
                    battery_cap_path = ''
                    battery_cap_val = ''

                try:
                    # NavigationAttitudeRoll
                    att_roll_path = x["updates"][0]["values"][6]["path"]
                    att_roll_val = x["updates"][0]["values"][6]["value"]
                    print(att_roll_path, "  :\t", att_roll_val)
                except IndexError:
                    att_roll_path = ''
                    att_roll_val = ''

                try:
                    # NavigationAttitudePitch
                    att_pitch_path = x["updates"][0]["values"][7]["path"]
                    att_pitch_val = x["updates"][0]["values"][7]["value"]
                    print(att_pitch_path, "  :\t", att_pitch_val)
                except IndexError:
                    att_pitch_path = ''
                    att_pitch_val = ''

                try:
                    # NavigationAttitudeYaw
                    att_yaw_path = x["updates"][0]["values"][8]["path"]
                    att_yaw_val = x["updates"][0]["values"][8]["value"]
                    print(att_yaw_path, "  :\t", att_yaw_val)
                except IndexError:
                    att_yaw_path = ''
                    att_yaw_val = ''

                try:
                    # NavigationAttitudeHeadingMagnetic
                    head_mag_path = x["updates"][0]["values"][9]["path"]
                    head_mag_val = x["updates"][0]["values"][9]["value"]
                    print(head_mag_path, "  :\t", head_mag_val)
                except IndexError:
                    head_mag_path = ''
                    head_mag_val = ''

                self.buffer = b''

            else:
                self.buffer += out


if __name__ == "__main__":
    ws = WindSensor()
    ws.read_sensor()








