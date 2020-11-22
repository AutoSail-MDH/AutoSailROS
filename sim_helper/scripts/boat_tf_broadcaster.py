#!/usr/bin/env python2.7
import rospy
import tf
import tf.transformations
from nav_msgs.msg import Odometry


def handle_boat_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((-msg.pose.pose.position.y, msg.pose.pose.position.x, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     'boat',
                     'map')


if __name__ == '__main__':
    rospy.init_node('boat_tf_broadcaster')
    rospy.Subscriber('/odom', Odometry, handle_boat_pose)
    rospy.spin()
