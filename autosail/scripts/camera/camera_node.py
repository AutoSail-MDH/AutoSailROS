#!/usr/bin/env python
import rospy
import math
import pyzed.sl as sl
import numpy as np
import cv2
import apriltag
import sensor_msgs.msg

from dynamic_reconfigure.server import Server
from autosail.cfg import CameraConfig
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge
from camera.bouydetection import detect_contour, startzedCamera, grab_frame, angle, convert_to_vec



def dynamic_reconf_callback(config, level):

    global B_L, G_L, R_L, B_U, G_U, R_U

    B_L = config.B_L
    G_L = config.G_L
    R_L = config.R_L

    B_U = config.B_U
    G_U = config.G_U
    R_U = config.R_U

    rospy.loginfo("Reconfigure request: {}".format(config))
    return config


if __name__ == "__main__":
    rospy.init_node("camera")
    refresh_rate = rospy.get_param("~rate", 60)
    queue_size = rospy.get_param("~queue_size", 1)
    camera_pub = rospy.Publisher(name="camera/data", data_class=Vector3Stamped, queue_size=queue_size)
    status_pub = rospy.Publisher(name="camera/status", data_class=String, queue_size=queue_size)
    apriltag_pub = rospy.Publisher(name="camera/apriltag", data_class=String, queue_size=queue_size)
    found_apriltags_pub = rospy.Publisher(name="camera/found_apriltags", data_class=String, queue_size=queue_size)
    image_pub = rospy.Publisher(name="camera/image", data_class=sensor_msgs.msg.Image, queue_size=queue_size)
    mask_pub = rospy.Publisher(name="camera/mask", data_class=sensor_msgs.msg.Image, queue_size=queue_size)

    # Dynamic reconfigure
    srv = Server(CameraConfig, dynamic_reconf_callback)

    rate = rospy.Rate(refresh_rate)
    zed, status = startzedCamera()

    # Check if camera initialized successfully
    if status != sl.ERROR_CODE.SUCCESS:
        # Delay for the publisher to have time to publish the message
        rospy.sleep(1/30)
        status_pub.publish(str(status))
        rospy.signal_shutdown("Error starting camera")

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
    # Setting the depth confidence parameters
    runtime_parameters.confidence_threshold = 100
    runtime_parameters.textureness_confidence_threshold = 100

    w2 = 1280 / 2  # Half of the camera resolution

    mirror_ref = sl.Transform()
    mirror_ref.set_translation(sl.Translation(2.75, 4.0, 0))
    tr_np = mirror_ref.m

    bridge = CvBridge()
    foundTag = set({"No AprilTag found"})
    tag = "No AprilTag found"

    while not rospy.is_shutdown():
        status_pub.publish(str(status))
        og_image, point_cloud = grab_frame(zed, runtime_parameters)
        x, y, image, cnt_image, mask = detect_contour(og_image)

        err, point_cloud_value = point_cloud.get_value(x, y)
        distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                             point_cloud_value[1] * point_cloud_value[1] +
                             point_cloud_value[2] * point_cloud_value[2])

        point_cloud_np = point_cloud.get_data()
        point_cloud_np.dot(tr_np)

        ang = angle(distance, x, w2)

        #Apriltag detection
        frame1 = cv2.cvtColor(og_image, cv2.COLOR_BGR2GRAY)


        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(frame1)
        #print("[INFO] {} total AprilTags detected".format(len(results)))




        if results != []:
            if "No AprilTag found" in foundTag:
                foundTag.remove("No AprilTag found")

            for r in results:
                tagFamily = r.tag_family.decode("utf-8")
                #print("[INFO] tag family: {}".format(tagFamily))
                foundTag.add(tagFamily)
                tag = tagFamily



        else:
            tag = "No AprilTag found"

        #print(foundTag)
        apriltag_pub.publish(str(tag))
        found_apriltags_pub.publish(str(foundTag))


        # If an objecd src/AutosailROSct is detected, publish its position as an x, y coordinate
        if not np.isnan(distance) and not np.isinf(distance):
            camera_data = Vector3Stamped()
            #camera_image = sensor_msgs.msg.Image()

            #camera_image.data = image
            object_coord_x, object_coord_y = convert_to_vec(distance, ang)
            camera_data.vector.x = object_coord_x
            camera_data.vector.y = object_coord_y
            camera_data.header.stamp = rospy.Time.now()
            camera_pub.publish(camera_data)
            #image_pub.publish(camera_image)

        img_msg = bridge.cv2_to_imgmsg(cnt_image, encoding='bgra8')
        image_pub.publish(img_msg)

        mask_img_msg = bridge.cv2_to_imgmsg(mask, encoding='bgra8')
        mask_pub.publish(mask_img_msg)
        rate.sleep()