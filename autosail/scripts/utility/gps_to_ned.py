#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped

velocity_pub = rospy.Publisher("/gps/fix_velocity", TwistWithCovarianceStamped, queue_size=1)


def gps_velocity_callback(data):
    global velocity_pub

    # Save x, y, and z from the gps velocity
    gps_x = data.twist.twist.linear.x
    gps_y = data.twist.twist.linear.y
    gps_z = data.twist.twist.linear.z

    # Convert the x, y, and z to NED
    x = gps_y
    y = gps_x
    z = -gps_z

    # Create and publish a TwistWithCovarianceStamped message
    velocity_msg = TwistWithCovarianceStamped()
    velocity_msg.header.stamp = rospy.Time.now()
    velocity_msg.twist.twist.linear.x = x
    velocity_msg.twist.twist.linear.y = y
    velocity_msg.twist.twist.linear.z = z

    velocity_pub.publish(velocity_msg)


if __name__ == "__main__":
    rospy.init_node("gps_to_ned")

    rospy.Subscriber("/gps/enu_velocity", TwistWithCovarianceStamped, gps_velocity_callback, queue_size=1)

    rospy.spin()
