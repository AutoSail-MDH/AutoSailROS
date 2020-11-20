#!/usr/bin/env python
import rospy
import math
from motor_controller.motor_controller import MotorController
from std_msgs.msg import Float64


class MotorControllerListener:
    def __init__(self):
        usbport = rospy.get_param("~/usbport", "/dev/ttyACM0")
        self.mcl = MotorController(usbport)
        self.rudder = None
        self.sail = None
        
    def rudder_callback(self, msg):
        # Store the message received.
        self.rudder = msg.data
        self.servo_control()

    def sail_callback(self, msg):
        # Store the message received.
        self.sail = msg.data
        self.servo_control()

    def servo_control(self):
        if self.sail is not None:
            angle = math.degrees(self.sail)  # takes the sail angle in radians and converts to degrees
            position = int(6.301*(angle + 80)+992)  # k value is 6.301 because input is between -45 and 45degrees
            rospy.loginfo("Position Sail:[%d]", position)  # logs the current position of the sail
            self.mcl.set_position(0, position)  # changes the position of servo 0
        if self.rudder is not None:
            angle = math.degrees(self.rudder)  # takes the rudder angle in radians and converts to degrees
            position = int(11.201*(angle + 45)+992)  # k value is 11.201 because input is between -80 and 80 degrees so
            # the slope between rudder and sail are different
            rospy.loginfo("Position Rudder:[%d]", position)  # logs the current position of the rudder
            self.mcl.set_position(1, position) # changes the position of servo 1


if __name__ == '__main__':
    rospy.init_node('motor_controller_listener')
    mcl = MotorControllerListener()
    queue_size = rospy.get_param("~/queue_size", 10)
    rospy.Subscriber('rudder_controller/rudder_angle', Float64, mcl.rudder_callback, queue_size=queue_size)
    rospy.Subscriber('sail_controller/sail_servo_angle', Float64, mcl.sail_callback, queue_size=queue_size)
    rospy.spin()
