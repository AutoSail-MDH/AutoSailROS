#!/usr/bin/env python3

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
        self.current_heading = data.orientation.z

    def callback_velocity(self, data):
        x = data.twist.twist.linear.x
        y = data.twist.twist.linear.y
        self.velocity = math.sqrt(math.pow(x, 2)+math.pow(y, 2))
        self.current_course = math.atan2(math.cos(x), math.sin(y))
        rospy.loginfo(x)


if __name__ == "__main__":
    rospy.init_node("rudder_controller")
    kp = rospy.get_param("~pid_coefficients/kp", 1)
    ki = rospy.get_param("~pid_coefficients/ki", 0.1)
    kd = rospy.get_param("~pid_coefficients/kd", 0.05)
    rc_pid = pid.PID(kp=kp, ki=ki, kd=kd)
    values = SubscriberValues()

    # Constants
    new_rudder_angle = 0
    if_heading_controller = True
    rudder_angle_limit = rospy.get_param("~rudder_limits", math.pi / 4)
    queue_size = rospy.get_param("~queue_size", 1)

    # Set update frequency
    refresh_rate = rospy.get_param("~rate", 60)
    r = rospy.Rate(refresh_rate)

    rospy.loginfo_once("queue_size: {}\n refresh_rate: {}\n kp: {}\n ki: {}\n kd: {}".format(queue_size, refresh_rate,
                                                                                             kp, ki, kd))

    # Subscribers
    rospy.Subscriber(name="path_planner/course", data_class=std_msgs.msg.Float32,
                     callback=values.callback_desired_course, queue_size=queue_size)  # The desired course
    rospy.Subscriber(name="/gps/navheading", data_class=sensor_msgs.msg.Imu,
                     callback=values.callback_current_heading, queue_size=queue_size)  # The heading
    rospy.Subscriber(name="gps/fix_velocity", data_class=geometry_msgs.msg.TwistWithCovarianceStamped,
                     callback=values.callback_velocity, queue_size=queue_size)  # The velocity and course

    # Publishers
    rudder_angle = rospy.Publisher(name="rudder_controller/rudder_angle", data_class=std_msgs.msg.Float32,
                                   queue_size=queue_size)

    while not rospy.is_shutdown():
        # Update the course given by the path planner
        rc_pid(values.desired_course)

        # Change between the course controller and the heading controller
        if rc.if_use_heading_as_setpoint(previous_bool=if_heading_controller, velocity=values.velocity):
            pid_heading = rc_pid(control_signal=values.current_heading)
            if_heading_controller = True
        else:
            pid_heading = rc_pid(control_signal=values.current_course)
            if_heading_controller = False
        new_rudder_angle = rc.rudder_angle_calculation(values.current_heading, pid_heading,
                                                       rudder_limit=rudder_angle_limit, velocity=values.velocity)

        # Publish the rudder angle
        rudder_angle.publish(new_rudder_angle)

        # Keep sync with the ROS frequency
        r.sleep()
