#!/usr/bin/env python
from rotary_sensor_spi.as5048 import AS5048
from periphery import SPI
import rospy
from std_msgs.msg import Float32, Int16
from std_srvs.srv import Trigger, TriggerResponse
from unittest.mock import create_autospec
import threading


def handle_write_zero_position(req):
    """
    A handle for the service to be able to handle write_zero_position

    :param req: The requester object which contains the input from the user
    :type req: object
    """
    lock.acquire()
    response = as5048a.write_zero_position()
    if as5048a.is_error:
        errors = as5048a.clear_and_get_error()
        lock.release()
        message = """Failed to write zero position, the following errors were encountered:
        Parity error: {}
        Command invalid: {}
        Framing error: {}""".format(**errors)
        return TriggerResponse(False, message)
    if as5048a.is_parity_mismatch:
        lock.release()
        message = "Failed to write zero position, the received package has a mismatched parity"
        return TriggerResponse(False, message)
    lock.release()
    old = AS5048.calc_angle(response['old_zero_position'])
    new = AS5048.calc_angle(response['new_zero_position'])
    message = """Successfully wrote a new zero position
    Old Zero Position: {} rad
    New Zero Position: {} rad""".format(old, new)
    return TriggerResponse(True, message)


def handle_read_diagnostics(req):
    """
    A handle for the service to be able to handle read_diagnostics

    :param req: The requester object which contains the input from the user
    :type req: Object
    """
    lock.acquire()
    response = as5048a.read_diagnostics()
    if as5048a.is_error:
        errors = as5048a.clear_and_get_error()
        lock.release()
        message = """Failed to read diagnostics, the following errors were encountered:
        Parity error: {}
        Command invalid: {}
        Framing error: {}""".format(**errors)
        return TriggerResponse(False, message)
    if as5048a.is_parity_mismatch:
        lock.release()
        message = "Failed to read diagnostics, the received package has a mismatched parity"
        return TriggerResponse(False, message)
    lock.release()
    message = """Diagnostics and Automatic Gain Control (AGC)
    Comp High: {comp_hi}
    Comp Low: {comp_lo}
    CORDIC OverFlow (COF): {cof}
    Offset Compensation Finished (OCF): {ocf}
    Automatic Gain Control value (AGC value): {agc_val}""".format(**response)
    return TriggerResponse(True, message)


def check_and_print_error(is_error, is_parity_mismatch):
    '''
    Checks if an error has occoured and prints a message

    :param is_error: A boolean if an error has occoured
    :type is_error: Boolean
    :param is_parity_mismatch: A boolean if an parity mismatch has happend on our end
    :type is_parity_mismatch: Boolean
    :return: True if an error is set, else false
    :rtype: Boolean
    '''
    errors = as5048a.clear_and_get_error()
    if as5048a.is_parity_mismatch:
        rospy.logerr('Received a broken package.')
        return True
    if as5048a.is_error:
        rospy.logerr("""Received a packet with error bit set, following errors are cleared;
                    parity error: {parity_error}
                    command invalid: {command_invalid}
                    framing error: {framing_error}""".format(**errors))
        return True
    return False


if __name__ == "__main__":
    lock = threading.Lock()  # Create thread lock to avoid race conditions
    rospy.init_node('as5048', anonymous=False)

    # Get parameters
    device = rospy.get_param('~spi/device', '/dev/spidev0.0')
    mode = rospy.get_param('~spi/mode', 1)
    max_hz = rospy.get_param('~spi/max_speed', 1000000)
    queue_size = rospy.get_param('~publisher_queue_size', 10)
    rate = rospy.get_param('~rate')
    create_mock = rospy.get_param('~create_mock', False)

    # Check if the mock parameter is set to run without a sensor
    if create_mock:
        spi = create_autospec(SPI)
        spi.transfer.return_value = [0, 0]
    else:
        spi = SPI(device, mode, max_hz)
    as5048a = AS5048(spi)
    # Setup publishers
    pub_angle = rospy.Publisher('angle_rad', Float32, queue_size=queue_size)
    pub_mag = rospy.Publisher('magnitude_raw', Int16, queue_size=queue_size)
    # Setup services
    s1 = rospy.Service('windvane_write_zero_position', Trigger, handle_write_zero_position)
    s2 = rospy.Service('windvane_read_diagnostics', Trigger, handle_read_diagnostics)
    # Apply node rate
    rate = rospy.Rate(rate)
    # Clear existing errors
    errors = as5048a.clear_and_get_error()
    if any(errors.values()):
        rospy.loginfo('Clear Error; Parity Error {parity_error}, Command Invalid {command_invalid}, Framing Error {framing_error}'.format(**errors))

    # Run main loop
    while not rospy.is_shutdown():
        lock.acquire()
        angle = as5048a.read_angle()
        if check_and_print_error(as5048a.is_error, as5048a.is_parity_mismatch):
            rospy.loginfo('Retrying to get angle package...')
            angle = as5048a.read_angle()
            if check_and_print_error(as5048a.is_error, as5048a.is_parity_mismatch):
                reason = 'Could not correct package problem'
                rospy.logfatal(reason)
                rospy.signal_shutdown(reason)
                exit()
            # TODO: Better error handling

        magnitude = as5048a.read_cordic_magnitude()
        if check_and_print_error(as5048a.is_error, as5048a.is_parity_mismatch):
            rospy.loginfo('Retrying to get magnitude package...')
            magnitude = as5048a.read_cordic_magnitude()
            if check_and_print_error(as5048a.is_error, as5048a.is_parity_mismatch):
                reason = 'Could not correct package problem'
                rospy.logfatal(reason)
                rospy.signal_shutdown(reason)
                exit()
            # TODO: Better error handling

        # Publish values and release
        pub_angle.publish(angle)
        pub_mag.publish(magnitude)
        lock.release()
        rospy.loginfo('Angle (rad): {}'.format(angle))

        rate.sleep()
    del as5048a
