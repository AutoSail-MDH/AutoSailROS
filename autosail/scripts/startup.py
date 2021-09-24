#!/usr/bin/env python
import os
import rospy
import roslaunch
import rospkg
import rostest

from ..test.integration_test import IntegrationTest


if __name__ == "__main__":
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    launch_file = os.path.join(rospkg.RosPack().get_path("autosail"), "launch", "system.launch")
    launch_system = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])

    rostest.rosrun("autosail", "integration_test", IntegrationTest)

    #launch_file = os.path.join(rospkg.RosPack().get_path("autosail"), "launch", "sensor.launch")
    #launch_drivers = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])

    try:
        launch_system.spin()
    except:
        #launch_drivers.shutdown()
        launch_system.shutdown()
