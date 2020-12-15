import serial
import time
import serial.tools.list_ports


ser = None


def openSTM32Serial(port):
    ser = serial.Serial(port, 52000, timeout=2)  # open serial port
    if ser.isOpen():
        print("STM32 port already open")
    else:
        ser.open()


def closeSTM32Serial():
    ser.close()


def sensor_readings():
    """
    Communicating to MCU to acquire sensor  data, from battery life, current consumption and water detection
    :return: data from MCU sensors
    """
    data = b'5'  # data sent
    ser.write(data)  # Serial port write data
    while True:
        data = ser.readline()
        if data != b"":
            break
    print("receive data is:", list(data))
    return data

