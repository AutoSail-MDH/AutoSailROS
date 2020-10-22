import numpy as np
import rospy


#  wind_sensor_readings[0] will give wind speed
#  wind_sensor_readings[1] will give wind angle in degrees
def sail_angle_calculation(wind_sensor_readings):
    #  limits on sail angle
    #  sail_limits = [-np.pi / 5.2, np.pi / 5.2]
    if -180 < wind_sensor_readings < 180:
        sail_limits = rospy.get_param('~sail_limits')
        sail_angle_rad = np.multiply(-np.sign(wind_sensor_readings),
                                     (((float(min(sail_limits)) - float(max(sail_limits))) / np.pi) *
                                      np.abs(np.deg2rad(wind_sensor_readings)) + max(sail_limits)))
    return sail_angle_rad
