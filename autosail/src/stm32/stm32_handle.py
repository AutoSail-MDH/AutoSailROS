import serial
import time
import serial.tools.list_ports


def sensor_readings():
    """
    Communicating to MCU to acquire sensor  data, from battery life, current consumption and water detection
    :return: data from MCU sensors
    """
    ser = serial.Serial('/dev/ttyACM60', 52000, timeout=2)  # open serial port
    if ser.isOpen():
        print("Already open")
    else:
        ser.open()
    data = b'5'  # data sent
    ser.write(data)  # Serial port write data
    while True:
        r = ser.read()
        data += r
        print(r)
        if r == b"\n":
            break
    print("receive data is:", list(data))
    ser.close()
    return data

