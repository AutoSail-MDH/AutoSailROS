import serial
import time
import serial.tools.list_ports


def sensor_readings():
    """
    Communicating to MCU to acquire sensor  data, from water level, current consumption and water detection
    :return: data from MCU sensors
    """
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=2)  # open serial port
    if ser.isOpen():
        print("Already open")
    else:
        ser.open()
    data = b'4'  # data sent
    ser.write(data)  # Serial port write data
    while True:
        data = ser.readline()
        if data != b"":
            break
    print("receive data is:", list(data))

    ser.close()
    return data

