import numpy as np


#  wind_sensor_readings[0] will give wind speed
#  wind_sensor_readings[1] will give wind angle in degrees
def sail_angle_calculation(wind_sensor_readings):
    #  limits on sail angle
    sail_limits = [-np.pi / 5.2, np.pi / 5.2]
    sail_angle_rad = np.multiply(-np.sign(wind_sensor_readings[1]),
                             (((min(sail_limits) - max(sail_limits)) / np.pi) *
                              np.abs(np.deg2rad(wind_sensor_readings[1])) + max(sail_limits)))
    return sail_angle_rad
