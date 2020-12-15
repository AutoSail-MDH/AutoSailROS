#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from path_planner.path_planner import quaternion_to_euler_yaw
import message_filters


def callback(wind_msg, imu_msg, vel_msg):
    lin_velocity_x = vel_msg.twist.twist.linear.x
    lin_velocity_y = vel_msg.twist.twist.linear.y
    velocity = [lin_velocity_x, lin_velocity_y]

    yaw = quaternion_to_euler_yaw(imu_msg.orientation)
    ang_velocity = imu_msg.angular_velocity

    wind = [wind_msg.vector.x, wind_msg.vector.y]
    v = [ang_velocity.x * 4.25, ang_velocity.y * 4.25]

    true_wind = [wind[0] + velocity[0] + v[0], wind[1] + velocity[1] + v[1]]
    true_wind = [np.cos(yaw) * true_wind[0] - np.sin(yaw) * true_wind[1],
                 np.sin(yaw) * true_wind[0] + np.cos(yaw) * true_wind[1]]

    true_wind_msg = Vector3Stamped()
    true_wind_msg.vector.x = true_wind[0]
    true_wind_msg.vector.y = true_wind[1]
    true_wind_msg.header.stamp = rospy.Time.now()
    pub.publish(true_wind_msg)


if __name__ == "__main__":
    pub = rospy.Publisher("wind_sensor/true", Vector3Stamped, queue_size=1)
    rospy.init_node("true_wind", log_level=rospy.get_param("log_level", rospy.INFO))

    fix_vel_sub = message_filters.Subscriber("/gps/fix_velocity", TwistWithCovarianceStamped)
    imu_sub = message_filters.Subscriber("/imu/data", Imu)
    wind_sub = message_filters.Subscriber("/wind_sensor/wind_vector", Vector3Stamped)
    ts = message_filters.ApproximateTimeSynchronizer([wind_sub, imu_sub, fix_vel_sub], 10, 0.1)
    ts.registerCallback(callback)

    rospy.spin()
