import numpy as np

sail_angle_rad = 0


#  wind_sensor_readings[0] will give wind speed
#  wind_sensor_readings[1] will give wind angle in degrees
def sail_angle_calculation(wind_sensor_readings, sail_limits):
    """
    Calculating the optimum angle the mast should be, dependant on the wind direction
    :param wind_sensor_readings: input from the wind sensor, to determine the direction the wind
    :param sail_limits: predefined values of maximum rotation of the wind to both sides (broadside, port side)
    :return: The angle in radians the sail should have according to the wind
    """
    global sail_angle_rad
    #  limits on sail angle
    #  sail_limits = [-np.pi / 5.2, np.pi / 5.2]

    if -181 < int(wind_sensor_readings) < 181:
        sail_angle_rad = np.multiply(-np.sign(wind_sensor_readings),
                                     (((float(-sail_limits) - float(sail_limits)) / np.pi) *
                                      np.abs(np.deg2rad(wind_sensor_readings)) + sail_limits))
    return sail_angle_rad
