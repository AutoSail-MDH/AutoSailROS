#!/usr/bin/env python
import rosunit
from motor_controller.motor_controller import MotorController
from unittest import TestCase, mock


class TestMotorController(TestCase):

    def setUp(self):
        self.patcher = mock.patch('motor_controller.motor_controller.serial.Serial')
        self.port = mock.patch('motor_controller.motor_controller.serial.tools.list_ports.comports')
        mock_port = self.port.start()
        mock_serial = self.patcher.start()
        mock_port.return_value = [["/dev/ttyACM0", "Test123"], ["/dev/ttyACM1", "Bazinga234"], ["/dev/ttyACM2", "Pololu"],
                                  ["/dev/ttyACM3", "Pololu"], ["/dev/ttyACM4", "Intel"], ["/dev/ttyACM5", "AMD"]]
        self.sc = MotorController()
        mock_serial.assert_called_with("/dev/ttyACM2", timeout=1)
        self.mock_serial = mock_serial.return_value

    def test_set_position(self):
        """
        Sets position to 1500 for both servos and verifies that the write function has been called with that value.
        """
        self.sc.set_position(0, 1500)
        self.mock_serial.write.assert_called_with([0xaa, 0x0c, 0x04, 0x00, 0x70, 0x2E])
        self.sc.set_position(1, 1500)
        self.mock_serial.write.assert_called_with([0xaa, 0x0c, 0x04, 0x01, 0x70, 0x2E])

    def test_close_servo(self):
        """
        Closes the serial connection to the motor controller and sets the two servos to their neutral position.
        """

        self.sc.close_servo()

        calls = [mock.call([0xaa, 0x0c, 0x04, 0x00, 0x70, 0x2E]),
                 mock.call([0xaa, 0x0c, 0x04, 0x01, 0x70, 0x2E]),
                 ]

        self.mock_serial.write.assert_has_calls(calls)

        self.mock_serial.close.assert_called_with()

    def test_set_angle(self):
        """
        Test different input angles and verifies that the function is working correctly.
        """
        self.sc.set_angle(0, 90)
        self.mock_serial.write.assert_called_with([0xFF, 0, int(254*90/180)])

        self.sc.set_angle(0, 185)
        self.mock_serial.write.assert_called_with([0xFF, 0, int(254 * 90 / 180)])

        self.sc.set_angle(0, -100)
        self.mock_serial.write.assert_called_with([0xFF, 0, int(254 * 90 / 180)])

        self.sc.set_angle(0, 45)
        self.mock_serial.write.assert_called_with([0xFF, 0, int(254 * 45 / 180)])

        self.sc.set_angle(1, 20)
        self.mock_serial.write.assert_called_with([0xFF, 1, int(254 * 20 / 180)])

    def test_get_position(self):
        """
        Checks that the get position function returns the position of the chosen servo with asserts.
        """
        self.mock_serial.read.side_effect = [chr(0x07), chr(0x0a)]
        position = self.sc.get_position(0)
        self.mock_serial.write.assert_called_with([0xaa, 0x0c, 0x10, 0])
        self.assertEqual(position, 2567/4)
        self.assertEqual(self.mock_serial.read.call_count, 2)

    def tearDown(self):
        self.patcher.stop()
        self.port.stop()


if __name__ == '__main__':
    rosunit.unitrun("autosail", "unittest_motor_controller", TestMotorController)



