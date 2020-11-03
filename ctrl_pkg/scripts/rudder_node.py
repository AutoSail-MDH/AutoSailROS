#!/usr/bin/env python

# Standard libs
import math

# Third party libs
import rospy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

# Local libs
from ctrl import rudder_controller as rc
from ctrl import pid


# Class for saving the values of the subscribers
class SubscriberValues:
    def __init__(self):
        self.desired_course = 0
        self.current_course = 0
        self.current_heading = 0
        self.velocity = 0

    def callback_desired_course(self, data):
        self.desired_course = data.data

    def callback_current_heading(self, data):
        # transform the quaternion to an Euler angle
        q = data.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

        self.current_heading = yaw

    def callback_velocity(self, data):
        x = data.twist.twist.linear.x
        y = data.twist.twist.linear.y
        self.velocity = math.sqrt(math.pow(x, 2)+math.pow(y, 2))
        self.current_course = math.atan2(math.sin(y), math.cos(x))


if __name__ == "__main__":
    rospy.init_node("rudder_controller")
    kp = rospy.get_param("~pid_coefficients/kp", 1)
    ki = rospy.get_param("~pid_coefficients/ki", 0.1)
    kd = rospy.get_param("~pid_coefficients/kd", 0.05)
    rc_pid = pid.PID(kp=kp, ki=ki, kd=kd)
    values = SubscriberValues()

    # Constants
    new_rudder_angle = 0
    rudder_angle_limit = rospy.get_param("~rudder_limits", math.pi / 4)
    queue_size = rospy.get_param("~queue_size", 1)
    upper_velocity_threshold = rospy.get_param("~rudder_upper_threshold", 1.5)
    lower_velocity_threshold = rospy.get_param("~rudder_lower_threshold", 0.5)

    # Set update frequency
    refresh_rate = rospy.get_param("~rate", 60)
    r = rospy.Rate(refresh_rate)

    # Subscribers
    rospy.Subscriber(name="/path_planner/course", data_class=std_msgs.msg.Float64,
                     callback=values.callback_desired_course, queue_size=queue_size)  # The desired course
    rospy.Subscriber(name="/gps/navheading", data_class=sensor_msgs.msg.Imu,
                     callback=values.callback_current_heading, queue_size=queue_size)  # The heading
    rospy.Subscriber(name="/gps/fix_velocity", data_class=geometry_msgs.msg.TwistWithCovarianceStamped,
                     callback=values.callback_velocity, queue_size=queue_size)  # The velocity and course

    # Publishers
    rudder_angle = rospy.Publisher(name="/rudder_controller/rudder_angle", data_class=std_msgs.msg.Float64,
                                   queue_size=queue_size)

    while not rospy.is_shutdown():
        # Update the course given by the path planner
        if rc_pid.setpoint != values.desired_course:
            rc_pid.setpoint = values.desired_course

        # Change between the course controller and the heading controller
        if rc.is_heading_setpoint(velocity=values.velocity, upper_threshold=upper_velocity_threshold,
                                  lower_threshold=lower_velocity_threshold):
            rospy.loginfo("Current heading: %f", values.current_heading)
            pid_heading = rc_pid(control_signal=values.current_heading)
        else:
            rospy.loginfo("Current course: %f", values.current_course)
            pid_heading = rc_pid(control_signal=values.current_course)
        new_rudder_angle = rc.calculate_rudder_angle(pid_corrected_heading=pid_heading,
                                                     rudder_limit=rudder_angle_limit)

        rospy.loginfo("Desired course: %f", values.desired_course)
        rospy.loginfo("Current velocity: %f", values.velocity)
        rospy.loginfo("PID output: %f", pid_heading)
        rospy.loginfo("Rudder angle: %f", new_rudder_angle)

        # Publish the rudder angle
        rudder_angle.publish(new_rudder_angle)

        # Keep sync with the ROS frequency
        r.sleep()
