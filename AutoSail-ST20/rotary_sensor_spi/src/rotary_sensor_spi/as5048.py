#!/usr/bin/env python
from __future__ import division  # Fix python 2 division problem
from periphery import SPI
import rospy
import math


class AS5048:

    CMD_READ = 0x4000
    ADDR_NOP = 0x0000
    ADDR_ANGLE = 0x3FFF
    ADDR_CLEAR_ERROR_FLAG = 0x0001
    ADDR_ZERO_POSITION_HI = 0x0016
    ADDR_ZERO_POSITION_LO = 0x0017
    ADDR_DIAGNOSTICS = 0x3FFD
    ADDR_MAGNITUDE = 0x3FFE

    is_error = 0
    is_parity_mismatch = 0

    def __init__(self, spi):
        rospy.loginfo('Opening SPI connection for AS5048A')
        self.spi = spi

    def __del__(self):
        rospy.loginfo('Closing SPI connection for AS5048A')
        self.spi.close()

    @staticmethod
    def check_parity_error(response):
        """
        Compares a response's parity bit with a calculated one from the rest of the bits of the response.
        Returns true when a mismatch is detected.

        :param response: A 16bit address response value
        :type response: 16bit Int
        :return: True if given parity is a mismatch to the calculated one, else false
        :rtype: Boolean
        """
        given_parity = response >> 15
        real_parity = AS5048.calc_even_parity(response & 0x7fff)
        return given_parity != real_parity

    @staticmethod
    def check_error_flag(response):
        """
        Extracts the error flag bit (14th bit) and returns it.

        :param response: A 16bit address response value
        :type response: 16bit Int
        :return: True if error flag is set, else false
        :rtype: Boolean
        """
        return ((response >> 14) & 0x1) == 1

    @staticmethod
    def calc_even_parity(addr):
        """
        Performs a XOR calculation to determine if the given 15bit address is even or not.
        Return 0 if the address has even bits, else 1.

        :param addr: A 15bit address value
        :type addr: 15bit Int
        :return: 0 if even, else 1
        :rtype: Int Boolean
        """
        for i in [8, 4, 2, 1]:
            addr ^= addr >> i
        return addr & 1

    @staticmethod
    def calc_angle(value):
        """
        Converts the given 13bit angle value to radians (0-2pi).

        :param value: 14bit angle value
        :type value: 14bit Int
        :return: Angle in radians between [0, 2pi]
        :rtype: Float
        """
        return value * (math.pi / 0x2000)

    def transfer(self, addr):
        """
        Adds a even parity bit on the given 15bit address. Sends the package as a 16bit address, and received a
        response from the previous transfer.
        If the response contains one of the following errors; error flag set, and parity mismatch,
        it will raise an error.

        :param addr: 15bit Address to send
        :type addr: 15bit Int
        :return: The 14bit data received
        :rtype: 14bit Int
        """
        # Prepare package
        addr |= self.calc_even_parity(addr) << 15  # Set parity at the 16th bit
        to_send = [(addr >> 8) & 0xff, addr & 0xff]  # Split the two bytes into an array
        # Send & receive package
        response = self.spi.transfer(to_send)
        rospy.logdebug('Sent: [0x{:02x}, 0x{:02x}], Received: [0x{:02x}, 0x{:02x}]'.format(*(to_send + response)))
        r = (response[0] << 8) | response[1]  # Concatenate the bytes
        # Verify parity and check error flag, set error to the corresponding value
        if self.check_error_flag(r):
            self.is_error = 1
        if self.check_parity_error(r):
            self.is_parity_mismatch = 1
        return r & 0x3fff  # Strip the parity and error bit and return only the data

    def read(self, addr):
        """
        Prepares the given address by setting the read bit (14th bit) to 1. Pass it to
        the transfer function to send it, and receives the data bits.

        :param addr: The 14bit address to send
        :type addr: 14bit Int
        :return: The 14bit data value from the response
        :rtype: 14bit Int
        """
        # Transfer the read address and then a NOP to get the result of the previous read.
        self.transfer(addr | self.CMD_READ) # Apply read bit before transfer
        result = self.transfer(self.ADDR_NOP | self.CMD_READ)
        return result

    def write(self, addr, data):
        """
        Send address as a write command (14th bit set to 0), skip the first response. Sends the data and receive
        the old register, and finally sends a NOP package to receive the new register.

        :param addr: The 14bit address to send
        :type addr: 14bit Int
        :param data: Data (14bit) to write
        :type data: Int
        :return: A dict with 'old_register' and 'new_register' values
        :rtype: Dict
        """
        self.transfer(addr)
        # First transfer will return the old register, and the second the new register
        old_register = self.transfer(data)
        new_register = self.transfer(self.ADDR_NOP | self.CMD_READ)
        return {'old_register': old_register, 'new_register': new_register}

    def read_angle(self):
        """
        Sends the angle address as a read, returns the angle in rad of the result.

        :return: The angle in radians
        :rtype: Float
        """
        addr = self.ADDR_ANGLE
        value = self.read(addr)
        return self.calc_angle(value)

    def read_cordic_magnitude(self):
        return self.read(self.ADDR_MAGNITUDE)

    def clear_and_get_error(self):
        """
        Send a clear error signal and receive what kind of error was set.

        :return: A dictionary containing the different type of errors
        :rtype: Dict
        """
        self.is_error = 0
        self.is_parity_mismatch = 0
        addr = self.ADDR_CLEAR_ERROR_FLAG
        response = self.read(addr)
        # Extract error bits
        parity_error = (response >> 2) & 0x1  # Extract parity error bit
        command_invalid = (response >> 1) & 0x1  # Extract command invalid bit
        framing_error = response & 0x1  # Extract framing error bit
        return {'parity_error': parity_error, 'command_invalid': command_invalid, 'framing_error': framing_error}

    def write_zero_position(self):
        """
        Write a new zero position by sending both high and low bits. Received a response containing
        both old and new register.

        :return: A dictionary containing the 'old_zero_position' and 'new_zero_position' value
        :rtype: Dict
        """
        response = self.read(self.ADDR_ANGLE)
        response_old_hi = self.read(self.ADDR_ZERO_POSITION_HI)
        response_old_lo = self.read(self.ADDR_ZERO_POSITION_LO)
        old = (response_old_hi << 6) | response_old_lo
        response = (response + old) % 0x4000

        # Extract zero position for high and low bits
        zero_position_hi = response >> 6
        zero_position_lo = response & 0b111111
        response_hi = self.write(self.ADDR_ZERO_POSITION_HI, zero_position_hi)
        response_lo = self.write(self.ADDR_ZERO_POSITION_LO, zero_position_lo)
        old = (response_hi['old_register'] << 6) | response_lo['old_register']
        new = (response_hi['new_register'] << 6) | response_lo['new_register']
        message = 'Wrote new zero position; old register 0x{:04x}, new register 0x{:04x}'.format(old, new)
        rospy.loginfo(message)
        return {'old_zero_position': old, 'new_zero_position': new}

    def read_diagnostics(self):
        """
        Sends a request to read the diagnostics of the sensor. Returns the diagnostics in a dictionary where
            - comp_high: Indicates a high magnetic field
            - comp_low: indiciates a weak magnetic field
            - cof: Bit is set if the CORDIC part indicated an out of range error,
                   which makes the angle and magnitude data invalid.
                   The last valid angular value will be used as the absolute output
            - ocf: Bit is set when the Offset Compensation Algorithm has finished. Always high after power up
            - agc_val: Represents the magnetic field value where 0 is high and 255 is low

        :return: The diagnostics
        :rtype: Dict
        """
        addr = self.ADDR_DIAGNOSTICS
        response = self.read(addr)
        # Extract diagnostic variables
        comp_hi = (response >> 11) & 1
        comp_lo = (response >> 10) & 1
        cof = (response >> 9) & 1
        ocf = (response >> 8) & 1
        agc_val = response & 0xFF
        return {'comp_hi': comp_hi, 'comp_lo': comp_lo, 'cof': cof, 'ocf': ocf, 'agc_val': agc_val}