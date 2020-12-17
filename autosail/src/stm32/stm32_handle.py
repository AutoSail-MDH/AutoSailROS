import serial
import time
import serial.tools.list_ports


ser = None


def openSTM32Serial(port):
    global ser
    ser = serial.Serial(port, 52000, timeout=2)  # open serial port
    if ser.isOpen():
        print("STM32 port already open")
        ser.close()
        ser.open()
    else:
        print("Opening STM32 port at: {}".format(port))
        ser.open()


def closeSTM32Serial():
    global ser
    ser.close()


def sensor_readings(timeout):
    """
    Communicating to MCU to acquire sensor  data, from battery life, current consumption and water detection
    :return: data from MCU sensors
    """
    global ser
    data = b'5'  # data sent
    ser.write(data)  # Serial port write data
    time_start = time.time()
    while True:
        data = ser.readline()
        if data != b"" or time.time() - time_start > timeout:
            break
    print("receive data is:", list(data))
    return data

