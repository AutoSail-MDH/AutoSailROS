#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger


def read_diagnostics_client():
    rospy.wait_for_service('windvane_read_diagnostics')
    try:
        read_diagnostics = rospy.ServiceProxy('windvane_read_diagnostics', Trigger)
        response = read_diagnostics()
        print(response.message)
    except rospy.ServiceException as err:
        print('Service call failed: %s'%err)


if __name__ == "__main__":
    print('Requesting to read diagnostics')
    read_diagnostics_client()