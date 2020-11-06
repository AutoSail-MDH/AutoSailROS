#!/usr/bin/env python
import math

import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from sensor_msgs.msg import Imu, NavSatFix

imu_pub = rospy.Publisher("gps/navheading", Imu, queue_size=1)
twist_pub = rospy.Publisher("gps/fix_velocity", TwistWithCovarianceStamped, queue_size=1)
wind_pub = rospy.Publisher("wind/apparent_rad", Float64, queue_size=1)
pos_pub = rospy.Publisher("gps/fix", NavSatFix, queue_size=1)
rudder_pub = rospy.Publisher("/rudder_position/command", Float64, queue_size=1)
# GPS fix position, imu quartenions, wind speed



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


desired_rudder_angle = 0.


def rudder_callback(data):
    global desired_rudder_angle
    desired_rudder_angle = data.data


if __name__ == "__main__":
    rospy.init_node("sim_converter")

    # Subscribes
    rospy.Subscriber(name="gazebo/model_states", data_class=ModelStates,
                     callback=model_state_callback, queue_size=1)
    rospy.Subscriber(name="wind/apparent", data_class=Vector3Stamped,
                     callback=wind_callback, queue_size=1)
    rospy.Subscriber(name="rudder_controller/rudder_angle", data_class=Float64, callback=rudder_callback, queue_size=1)
    r = rospy.Rate(260)
    current_rudder_angle = 0.
    onedeginrad = math.pi/180.
    while not rospy.is_shutdown():
        if abs(desired_rudder_angle-current_rudder_angle) <= onedeginrad:
            current_rudder_angle = desired_rudder_angle
        else:
            current_rudder_angle += math.copysign(onedeginrad, desired_rudder_angle-current_rudder_angle)
        rudder_pub.publish(current_rudder_angle)
        r.sleep()
    rospy.spin()
