#!/usr/bin/env python
import time
import serial
import serial.tools.list_ports


class MotorController:

    def __init__(self):
        self.sc = None
        ports = list(serial.tools.list_ports.comports())
        port = None
        for p in ports:
            if p[1].startswith("Pololu"):
                pnr = int(p[0][-1])
                if port is None or pnr < int(port[-1]):
                    port = p[0]
        print(port)
        if port is None:
            raise Exception("Could not find port")
        # Tries 10 times before exiting
        tries = 10
        for i in range(tries):
            try:
                self.sc = serial.Serial(port, timeout=1)
            except serial.SerialException as e:
                time.sleep(1)
                if i == tries-1:
                    raise e
                continue
            break

    def __del__(self):
        self.close_servo()

    def close_servo(self):
        """
        Closes the serial communication and sets both our servos to their default position.
        """
        if self.sc is not None and self.sc.isOpen() == True:
            self.set_position(0, 1500)
            self.set_position(1, 1500)
            self.sc.close()

    def set_angle(self, servo, angle):
        """
        Takes an angle given by the user and imposes it on the chosen servo(0->n-1 where n is the number of servos the controller is capable of handling)
        @param servo: Is the ID of the servo you want to control
        @type servo: Integer
        @param angle: The angle 0-360
        @type angle: Integer
        """
        if angle > 180 or angle < 0:
            angle = 90
        byteone = int(254*angle/180)
        bud = [0xFF, servo, byteone]
        self.sc.write(bud)

    def set_position(self, servo, position):
        """
        Sets the position of the chosen servo(0->n-1 where n is the number of servos the controller is capable of handlin
        g) with the given position(992-2000)(low to high) in microSeconds.
        @param servo: Is the ID of the servo you want to control
        @type servo: Integer
        @param position: The PWM signal in micro seconds going from 992 to 2000
        @type position: Integer
        """
        position = position * 4
        posLow = (position & 0x7f)
        posHigh = (position >> 7) & 0x7f
        chan = servo & 0x7f
        data = [0xaa, 0x0c, 0x04, chan, posLow, posHigh]
        self.sc.write(data)

    def get_position(self, servo):
        """
        Returns two bytes with the current position of the servo. For example a position of 2567 corresponds to the re
        sponse 0x07, 0x0A where 2567 = 0xA07(second byte is shifted then added). This position value is four times the 
        number usually displayed in the Maestro software(992-2000). 
        @param servo:Is the ID of the servo you want to check
        @type servo: Integer
        @return: w1 and w2 which are the low 8bits and high 8bits respectively
        @rtype: Integer
        """
        chan = servo & 0x7f
        data = [0xaa, 0x0c, 0x10, chan]
        self.sc.write(data)
        w1 = ord(self.sc.read())
        w2 = ord(self.sc.read())
        w2 = w2 << 8
        w3 = (w1 | w2) / 4
        return w3

    def get_errors(self):
        data = [0xaa, 0x0c, 0x21]
        self.sc.write(data)
        w1 = ord(self.sc.read())
        w2 = ord(self.sc.read())
        return w1, w2

    """
    def triggerScript(self, subNumber):
        data = chr(0xaa) + chr(0x0c) + chr(0x27) +chr(0)
        self.sc.write(data)
        """


if __name__ == "__main__":
    mm = MotorController()
    mm.set_position(0, 1000)
    pos = mm.get_position(0)
    print(f"Servo position: {pos}")
    mm.sc.close()

