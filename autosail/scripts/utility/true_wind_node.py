#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from path_planner.path_planner import quaternion_to_euler_yaw

velocity = None
yaw = None
wind = None


def gps_velocity_callback(data):
    global velocity
    lin_velocity_x = data.twist.twist.linear.x
    lin_velocity_y = - data.twist.twist.linear.y
    velocity = [lin_velocity_x, lin_velocity_y]


def imu_heading_callback(data):
    global yaw
    yaw = -quaternion_to_euler_yaw(data.orientation)
    #yaw = [np.cos(yaw), np.sin(yaw)]


def wind_callback(data):
    global wind
    wind = [data.vector.x, -data.vector.y]


if __name__ == "__main__":
    rospy.init_node("true_wind")
    rospy.Subscriber("/gps/fix_velocity", TwistWithCovarianceStamped, gps_velocity_callback, queue_size=1)
    rospy.Subscriber("/imu/data", Imu, imu_heading_callback, queue_size=1)
    rospy.Subscriber("/wind_sensor", Vector3Stamped, wind_callback, queue_size=1)
    pub = rospy.Publisher("wind_sensor/true", Vector3Stamped, queue_size=1)

    rate = rospy.Rate(8)
    while not rospy.is_shutdown():
        if wind is not None and yaw is not None and velocity is not None:
            true_wind = [wind[0] + velocity[0], wind[1] + velocity[1]]
            true_wind = [np.cos(yaw)*true_wind[0] - np.sin(yaw)*true_wind[1],
                         np.sin(yaw)*true_wind[0] + np.cos(yaw)*true_wind[1]]

            true_wind_msg = Vector3Stamped()
            true_wind_msg.vector.x = true_wind[0]
            true_wind_msg.vector.y = true_wind[1]
            true_wind_msg.header.stamp = rospy.Time.now()
            pub.publish(true_wind_msg)
            rate.sleep()