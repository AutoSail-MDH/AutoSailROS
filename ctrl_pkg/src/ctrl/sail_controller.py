def sail_angle_calculation(wind_sensor_readings):
    D_sail = np.multiply(-np.sign(W_apparent), (((min(Sail) - max(Sail)) / np.pi) * np.abs(W_apparent) + max(Sail)))
    return sail_angle