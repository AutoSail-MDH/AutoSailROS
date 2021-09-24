#!/usr/bin/env python
from unittest import TestCase, mock
from unittest.mock import call
import math
from rotary_sensor_spi.as5048 import AS5048

PKG = 'rotary_sensor_spi'
NAME = 'test_as5048'

class TestAS5048A(TestCase):
    def test_check_parity_error(self):
        self.assertTrue(AS5048.check_parity_error(0b1001001001010000))
        self.assertTrue(AS5048.check_parity_error(0b0010110101100100))
        self.assertFalse(AS5048.check_parity_error(0b1000010001010000))
        self.assertFalse(AS5048.check_parity_error(0b0000110101100001))

    def test_check_error_flag(self):
        self.assertTrue(AS5048.check_error_flag(0b0100100101100001))
        self.assertFalse(AS5048.check_error_flag(0b0000100101100001))

    def test_calc_even_parity(self):
        # The parity should alternate when removing the MSB
        p = 1
        for i in range(0, 15):
            self.assertEqual(AS5048.calc_even_parity(0x7fff >> i), p)
            p = 1 - p

        # Random tests
        self.assertEqual(AS5048.calc_even_parity(0b000010000010000), 0)
        self.assertEqual(AS5048.calc_even_parity(0b000010010010000), 1)
        self.assertEqual(AS5048.calc_even_parity(0b001010011000101), 0)
        self.assertEqual(AS5048.calc_even_parity(0b101100101001011), 0)
        self.assertEqual(AS5048.calc_even_parity(0b011001100010000), 1)
        self.assertEqual(AS5048.calc_even_parity(0b11110000010000), 1)
        self.assertEqual(AS5048.calc_even_parity(0b001011011110100), 0)
        self.assertEqual(AS5048.calc_even_parity(0b110000110010111), 0)
        self.assertEqual(AS5048.calc_even_parity(0b110111100110101), 0)
        self.assertEqual(AS5048.calc_even_parity(0b101010101010101), 0)
        self.assertEqual(AS5048.calc_even_parity(0b010101010101010), 1)

    def test_calc_angle(self):
        self.assertEqual(AS5048.calc_angle(0), 0)
        self.assertEqual(AS5048.calc_angle(0x1000), math.pi / 2)
        self.assertEqual(AS5048.calc_angle(0x2000), math.pi)
        self.assertEqual(AS5048.calc_angle(0x3000), math.pi * 3 / 2)
        self.assertEqual(AS5048.calc_angle(0x4000), 2 * math.pi)

    @mock.patch('rotary_sensor_spi.as5048.SPI')
    def test_read(self, mock_spi):
        as5048 = AS5048(mock_spi)
        # Check so read bit is set
        with mock.patch.object(AS5048, 'transfer', return_value=0x3fff) as mock_transfer:
            as5048.read(AS5048.ADDR_NOP)
            self.assertEqual(mock_transfer.call_count, 2)
            calls = [call(0x4000), call(0x4000)]
            mock_transfer.assert_has_calls(calls)
            mock_transfer.reset_mock()
            as5048.read(AS5048.ADDR_ANGLE)
            self.assertEqual(mock_transfer.call_count, 2)
            calls = [call(0x7fff), call(0x4000)]
            mock_transfer.assert_has_calls(calls)

        # Test default operation of read without any errors
        mock_spi.transfer.side_effect = [[0x00, 0x00], [0x3f, 0xff]]
        response = as5048.read(AS5048.ADDR_ANGLE)
        # Check calls
        self.assertEqual(mock_spi.transfer.call_count, 2)
        calls = [call([0xff, 0xff]), call([0xc0, 0x00])]
        mock_spi.transfer.assert_has_calls(calls)
        # Check final returns
        self.assertEqual(response, 0x3fff, 'Wrong response from read, should be the same as the transfer return')
        self.assertFalse(as5048.is_error, 'Error is set')
        self.assertFalse(as5048.is_parity_mismatch, 'Parity mismatch')

    @mock.patch('rotary_sensor_spi.as5048.SPI')
    def test_write(self, mock_spi):
        as5048 = AS5048(mock_spi)
        # Test default operation of write without any errors
        mock_spi.transfer.side_effect = [[0x00, 0x00], [0x01, 0x02], [0x80, 0x04]]
        response = as5048.write(AS5048.ADDR_ZERO_POSITION_HI, 0x0004)
        # Check calls
        self.assertEqual(mock_spi.transfer.call_count, 3)
        calls = [call([0x80, 0x16]), call([0x80, 0x04]), call([0xc0, 0x00])]
        mock_spi.transfer.assert_has_calls(calls)
        # Check final returns
        self.assertEqual(response['old_register'], 0x0102)
        self.assertEqual(response['new_register'], 0x0004)
        self.assertFalse(as5048.is_error, 'Error is set')
        self.assertFalse(as5048.is_parity_mismatch, 'Parity mismatch')


    @mock.patch('rotary_sensor_spi.as5048.SPI')
    def test_transfer_and_clear_error(self, mock_spi):
        as5048 = AS5048(mock_spi)
        # Normal behaviour without error
        mock_spi.transfer.return_value = [0x3f, 0xff]
        value = as5048.transfer(0x3fff)
        mock_spi.transfer.assert_called_once_with([0x3f, 0xff])
        self.assertEqual(value, 0x3fff)
        self.assertFalse(as5048.is_error, 'Error is set')
        self.assertFalse(as5048.is_parity_mismatch, 'Parity is mismatched')
        # Receives a parity mismatch from the spi
        mock_spi.reset_mock()
        mock_spi.transfer.return_value = [0xbf, 0xff]
        value = as5048.transfer(0xbfff)
        mock_spi.transfer.assert_called_once_with([0xbf, 0xff])  # 0xffff since a even parity should be set
        self.assertEqual(value, 0x3fff)
        self.assertFalse(as5048.is_error, 'Error is set')
        self.assertTrue(as5048.is_parity_mismatch, 'Parity is not mismatched')
        # Clear errors
        mock_spi.reset_mock()
        mock_spi.transfer.return_value = [0, 3]
        errors = as5048.clear_and_get_error()
        self.assertEqual(mock_spi.transfer.call_count, 2)
        self.assertDictEqual(errors, {'parity_error': 0, 'command_invalid': 1, 'framing_error': 1})
        self.assertFalse(as5048.is_error)
        self.assertFalse(as5048.is_parity_mismatch)
        # SPI returns with an error bit set
        mock_spi.reset_mock()
        mock_spi.transfer.return_value = [0x7f, 0x1f]
        value = as5048.transfer(0x0001)
        mock_spi.transfer.assert_called_once_with([0x80, 0x01])
        self.assertEqual(value, 0x3f1f)
        self.assertTrue(as5048.is_error, 'Error is not set')
        self.assertFalse(as5048.is_parity_mismatch, 'Parity is mismatched')
        # Clear errors
        mock_spi.reset_mock()
        mock_spi.transfer.return_value = [0x80, 4]
        errors = as5048.clear_and_get_error()
        self.assertDictEqual(errors, {'parity_error': 1, 'command_invalid': 0, 'framing_error': 0})
        self.assertFalse(as5048.is_error)
        self.assertFalse(as5048.is_parity_mismatch)
        # Parity mismatch and error bit set
        mock_spi.reset_mock()
        mock_spi.transfer.return_value = [0xff, 0x45]
        value = as5048.transfer(0xffff)
        mock_spi.transfer.assert_called_once_with([0xff, 0xff])
        self.assertEqual(value, 0x3f45)
        self.assertTrue(as5048.is_error, 'Error is not set')
        self.assertTrue(as5048.is_parity_mismatch, 'Parity is not mismatched')
        # Make another call with error still set
        mock_spi.reset_mock()
        mock_spi.transfer.return_value = [0x3f, 0xff]
        value = as5048.transfer(0xffff)
        mock_spi.transfer.assert_called_once_with([0xff, 0xff])
        self.assertTrue(mock_spi.transfer.called)
        self.assertEqual(value, 0x3fff)
        self.assertTrue(as5048.is_error, 'Error is not set')
        self.assertTrue(as5048.is_parity_mismatch, 'Parity is mismatched')
        # Clear errors
        mock_spi.reset_mock()
        mock_spi.transfer.return_value = [0, 0]
        errors = as5048.clear_and_get_error()
        self.assertDictEqual(errors, {'parity_error': 0, 'command_invalid': 0, 'framing_error': 0})
        self.assertFalse(as5048.is_error)
        self.assertFalse(as5048.is_parity_mismatch)

    @mock.patch('rotary_sensor_spi.as5048.SPI')
    def test_read_angle(self, mock_spi):
        mock_spi.transfer.return_value = [0x00, 0x00]
        as5048 = AS5048(mock_spi)
        angle = as5048.read_angle()
        mock_spi.transfer.assert_has_calls([call([0xff, 0xff]), call([0xc0, 0x00])])
        self.assertEqual(angle, 0)

        mock_spi.transfer.return_value = [0x3f, 0xff]
        angle = as5048.read_angle()
        self.assertAlmostEqual(angle, 6, delta=0.3)

    @mock.patch('rotary_sensor_spi.as5048.SPI')
    def test_write_zero_position(self, mock_spi):
        mock_spi.transfer.side_effect = [[0, 0], [0, 3],  # read angle
                                          [0, 0], [0, 0],  # read pos high
                                          [0, 0], [0, 0],  # read pos low
                                          [0, 0], [0, 0], [0, 0],  # write pos high
                                          [0, 0], [0, 0], [0, 3]]  # write pos low
        as5048 = AS5048(mock_spi)
        response = as5048.write_zero_position()
        self.assertEqual(mock_spi.transfer.call_count, 12)
        self.assertDictEqual(response, {'old_zero_position': 0, 'new_zero_position': 3})
        self.assertFalse(as5048.is_error)
        self.assertFalse(as5048.is_parity_mismatch)

    @mock.patch('rotary_sensor_spi.as5048.SPI')
    def test_read_diagnostics(self, mock_spi):
        mock_spi.transfer.return_value = [0b1010, 0b10011010]
        as5048 = AS5048(mock_spi)
        response = as5048.read_diagnostics()
        self.assertEqual(mock_spi.transfer.call_count, 2)
        self.assertDictEqual(response, {'comp_hi': 1, 'comp_lo': 0, 'cof': 1, 'ocf': 0, 'agc_val': 0b10011010})
        self.assertFalse(as5048.is_error)
        self.assertFalse(as5048.is_parity_mismatch)

    @mock.patch('rotary_sensor_spi.as5048.SPI')
    def test_read_cordic_magnitude(self, mock_spi):
        mock_spi.transfer.return_value = [0x00, 0x00]
        as5048 = AS5048(mock_spi)
        magnitude = as5048.read_cordic_magnitude()
        mock_spi.transfer.assert_has_calls([call([0x7f, 0xfe]), call([0xc0, 0x00])])
        self.assertEqual(magnitude, 0)

        mock_spi.transfer.return_value = [0x3f, 0xff]
        magnitude = as5048.read_cordic_magnitude()
        self.assertEqual(magnitude, 0x3fff)

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG, NAME, TestAS5048A)