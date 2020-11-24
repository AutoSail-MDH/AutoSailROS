#!/usr/bin/env python
import rospy
import math
import pyzed.sl as sl

from std_msgs.msg import Int64
from bouydetection import detect_contour, startzedCamera, grab_frame, angle


if __name__ == "__main__":
    rospy.init_node("camera")
    camera_pub = rospy.Publisher(name="camera/data", data_class=Int64, queue_size=1)
    rate = rospy.Rate(60)

    zed = startzedCamera()

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
    # Setting the depth confidence parameters
    runtime_parameters.confidence_threshold = 100
    runtime_parameters.textureness_confidence_threshold = 100

    w2 = 1280/2  # Half of the camera resolution

    mirror_ref = sl.Transform()
    mirror_ref.set_translation(sl.Translation(2.75, 4.0, 0))
    tr_np = mirror_ref.m

    while not rospy.is_shutdown():
        image, point_cloud = grab_frame(zed, runtime_parameters)
        x, y = detect_contour(image)

        err, point_cloud_value = point_cloud.get_value(x, y)
        distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                             point_cloud_value[1] * point_cloud_value[1] +
                             point_cloud_value[2] * point_cloud_value[2])

        point_cloud_np = point_cloud.get_data()
        point_cloud_np.dot(tr_np)

        ang = angle(distance, x, w2)

        rate.sleep()
