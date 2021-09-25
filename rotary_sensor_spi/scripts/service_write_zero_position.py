#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger


def write_zero_position_client():
    rospy.wait_for_service('windvane_write_zero_position')
    try:
        write_zero_position = rospy.ServiceProxy('windvane_write_zero_position', Trigger)
        response = write_zero_position()
        print(response.message)
    except rospy.ServiceException as err:
        print('Service call failed: %s'%err)


if __name__ == "__main__":
    print('Requesting to write a new zero position at the current position')
    write_zero_position_client()