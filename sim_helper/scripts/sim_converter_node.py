#!/usr/bin/env python
import math

import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from sensor_msgs.msg import Imu

imu_pub = rospy.Publisher("gps/navheading", Imu, queue_size=1)
twist_pub = rospy.Publisher("gps/fix_velocity", TwistWithCovarianceStamped, queue_size=1)
wind_pub = rospy.Publisher("wind/apparent_rad", Float64, queue_size=1)


def model_state_callback(data):
    global imu_pub, twist_pub
    i = data.name.index("rs750")
    linear = data.twist[i].linear
    angular = data.twist[i].angular
    q = data.pose[i].orientation

    imu_msg = Imu()
    imu_msg.orientation = q
    imu_pub.publish(imu_msg)

    twist_msg = TwistWithCovarianceStamped()
    twist_msg.twist.twist.linear = linear
    twist_msg.twist.twist.angular = angular
    twist_pub.publish(twist_msg)


def wind_callback(data):
    global wind_pub
    wind_angle = math.atan2(data.vector.y, data.vector.x)
    wind_pub.publish(wind_angle)


if __name__ == "__main__":
    rospy.init_node("sim_converter")

    # Subscribes
    rospy.Subscriber(name="gazebo/model_states", data_class=ModelStates,
                     callback=model_state_callback, queue_size=1)
    rospy.Subscriber(name="wind/apparent", data_class=Vector3Stamped,
                     callback=wind_callback, queue_size=1)
    rospy.spin()