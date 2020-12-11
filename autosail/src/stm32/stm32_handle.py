import serial
import time
import serial.tools.list_ports


def sensor_readings():
    """
    Communicating to MCU to acquire sensor  data, from battery life, current consumption and water detection
    :return: data from MCU sensors
    """
<<<<<<< HEAD
    ser = serial.Serial('/dev/ttyACM60', 52000, timeout=0.5)  # open serial port
=======
    ser = serial.Serial('/dev/ttyACM60', 52000, timeout=2)  # open serial port
>>>>>>> 7ce56298cc804060c2007d53e467a692153b1551
    if ser.isOpen():
        print("Already open")
    else:
        ser.open()
    data = b'5'  # data sent
    ser.write(data)  # Serial port write data
    while True:
        data = ser.readline()
        if data != b"":
            break
    print("receive data is:", list(data))
    ser.close()
    return data

