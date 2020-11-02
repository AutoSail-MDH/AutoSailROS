import numpy as np

sail_angle_rad = 0

def calculate_sail_angle(wind_sensor_readings, sail_limits):
    """
    Calculating the optimum angle the mast should be, dependant on the wind direction
    :param wind_sensor_readings: input from the wind sensor, to determine the direction the wind
    :param sail_limits: predefined values of maximum rotation of the wind to both sides (broadside, port side)
    :return: The angle in radians the sail should have according to the wind
    """
    global sail_angle_rad

    if -np.pi <= wind_sensor_readings < np.pi:
        sail_angle_rad = np.multiply(-np.sign(wind_sensor_readings),
                                     (((float(-sail_limits) - float(sail_limits)) / np.pi) *
                                      np.abs(wind_sensor_readings) + sail_limits))
    return sail_angle_rad


def trim_sail(sail_angle, sail_limits, scalar):
    """
    Using the sail angle to adjust the servo which is trimming the sail
    :param sail_angle: angle of the sail
    :return: the amount of degree the servo should be positioned in 1620 loos sail and 0 beeing maximum trim
    """
    if sail_angle > sail_limits:
        return 0
    elif sail_angle < 0:
        return sail_limits * scalar
    servo_degree = np.multiply(sail_limits-np.abs(sail_angle), scalar)
    return np.abs(servo_degree)
