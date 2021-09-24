#!/usr/bin/env python
import sys
import rospy
from unittest import TestCase
from unittest.mock import patch
from std_srvs.srv import Trigger

PKG = 'rotary_sensor_'
NAME = 'test_as5048_service'


class TestAS5048Service(TestCase):

    def test_write_zero_position_service(self):
        rospy.wait_for_service('windvane_write_zero_position')
        write_zero_position = rospy.ServiceProxy('windvane_write_zero_position', Trigger)
        response = write_zero_position()
        self.assertTrue(response.success)
        self.assertTrue(response.message.startswith('Successfully '))

    def test_read_diagnostics_service(self):
        rospy.wait_for_service('windvane_read_diagnostics')
        read_diagnostics = rospy.ServiceProxy('windvane_read_diagnostics', Trigger)
        response = read_diagnostics()
        self.assertTrue(response.success)
        self.assertTrue(response.message.startswith('Diagnostics '))


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestAS5048Service, sys.argv)