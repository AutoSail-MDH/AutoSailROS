#!/usr/bin/env python3

# Standard libs
import math

# Third party libs
import rospy
import std_msgs.msg

# Local libs
from ctrl import rudder_controller as rc
from ctrl import pid


# Class for saving the values of the subscribers
class SubscriberValues:
    def __init__(self):
        self.desired_course = 0
        self.current_course = 0
        self.current_heading = 0
        self.velocity_flag = 0

    def callback_course(self, data):
        if data.data is not None:
            self.current_course = data.data

    def callback_heading(self, data):
        if data.data is not None:
            self.current_heading = data.data

    def callback_velocity_flag(self, data):
        if data.data is not None:
            self.velocity_flag = data.data

    def callback_desired_course(self, data):
        if data.data is not None:
            self.desired_course = data.data


if __name__ == "__main__":
    rospy.init_node("pid")
    kp = rospy.get_param("~pid_coefficients/kp", 1)
    ki = rospy.get_param("~pid_coefficients/ki", 0.1)
    kd = rospy.get_param("~pid_coefficients/kd", 0.05)
    rc_pid = pid.PID(kp=kp, ki=ki, kd=kd)
    values = SubscriberValues()

    # Constants
    new_rudder_angle = 0
    if_heading_controller = True
    rudder_angle_limit = rospy.get_param("~rudder_limits", math.pi/4)

    # Set update frequency
    refresh_rate = rospy.get_param("~rate", 60)
    r = rospy.Rate(refresh_rate)

    # Temporary testing publishers TODO: remove
    pub_test1 = rospy.Publisher("pathplanner", std_msgs.msg.Float32, queue_size=1)
    pub_test_current_course = rospy.Publisher("current_course", std_msgs.msg.Float32, queue_size=1)
    pub_test_current_heading = rospy.Publisher("current_heading", std_msgs.msg.Float32, queue_size=1)
    pub_test4 = rospy.Publisher("velocity_above_threshold", std_msgs.msg.Bool, queue_size=1)

    # Subscribers
    rospy.Subscriber(name="current_course", data_class=std_msgs.msg.Float32, callback=values.callback_course,
                     queue_size=1)  # TODO: change the name to what is used by the current course publishing
    rospy.Subscriber(name="current_heading", data_class=std_msgs.msg.Float32, callback=values.callback_heading,
                     queue_size=1)  # TODO: change the name of topic
    rospy.Subscriber(name="velocity", data_class=std_msgs.msg.Bool, callback=values.callback_velocity_flag,
                     queue_size=1)  # TODO: change the name

    # Publishers
    rudder_angle = rospy.Publisher(name="rudder_angle", data_class=std_msgs.msg.Float32, queue_size=1)

    # Parameters for testing TODO: remove
    i = 0
    v = 5
    v_c = 1
    i_c = 0.1
    a = 0
    b = 0

    while not rospy.is_shutdown():
        # Update the course given by the path planner
        rc_pid(values.desired_course)

        # Change between the course controller and the heading controller
        if rc.if_use_heading_as_setpoint(previous_bool=if_heading_controller, velocity=v):
            pid_heading = rc_pid(control_signal=values.current_heading)
            if_heading_controller = True
        else:
            pid_heading = rc_pid(control_signal=values.current_course)
            if_heading_controller = False
        new_rudder_angle = rc.rudder_angle_calculation(values.current_heading, pid_heading,
                                                       rudder_limit=rudder_angle_limit, velocity=v)

        # Publish the rudder angle
        rudder_angle.publish(new_rudder_angle)

        # Keep sync with the ROS frequency
        r.sleep()

        # Temporary testing TODO: remove
        pub_test_current_heading.publish(a)
        pub_test_current_course.publish(0)

        if i >= 1:
            i_c = -0.1
            a = math.pi
            b = math.pi
        elif i <= -1:
            i_c = 0.1
            a = -math.pi
            b = -math.pi
        i = round(i + i_c, 1)

        if v >= 10:
            v_c = -1
        elif v <= 0:
            v_c = 1
        v = 10
        # v = round(v + v_c)

