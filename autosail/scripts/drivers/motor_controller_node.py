#!/usr/bin/env python
import rospy
import math
from motor_controller.motor_controller import MotorController
from std_msgs.msg import Float64


class MotorControllerListener:
    def __init__(self):
        '''
        initialization of the MotorControllerListener, rudder and sail are set to None as a starting value.
        '''
        self.mcl = MotorController()
        self.rudder = None
        self.sail = None
        rudder_limit = rospy.get_param("~rudder_servo_limit", 90)
        sail_limit = rospy.get_param("~sail_servo_limit", 1440)
        self.rudder_mid = rudder_limit/2
        self.rudder_k = 1008/rudder_limit
        self.sail_k = 1008/sail_limit
        
    def rudder_callback(self, msg):
        '''
        Function to read what the rudder controllers sends through the publisher and use the servo_control() function
        to actuate said reading onto the servos.
        :param msg: Value from the rudder controllers publisher that is in radians between -45 and 45 degrees
        :type msg: Integer
        '''
        # Store the message received.
        self.rudder = msg.data+math.radians(5)
        self.servo_control()

    def sail_callback(self, msg):
        '''
        Function to read what the sail controllers sends through the publisher and use the servo_control() function
        to actuate said reading onto the servos.
        :param msg: Value from the sail controllers publisher that is in radians between 0 and 1620 degrees
        :type msg: Integer
        '''
        # Store the message received.
        self.sail = msg.data
        self.servo_control()

    def servo_control(self):
        '''
        Function that is called by sail_callback and rudder_callback to actuate the desired angle onto the servos
        '''
        if self.sail is not None:
            angle = math.degrees(self.sail)  # takes the sail angle in radians and converts to degrees
            # k value is 0.7 because input is between 0 and 1440 degrees so
            position = int(self.sail_k*angle+992)
            rospy.loginfo("Position Sail:[%d]", position)  # logs the current position of the sail
            self.mcl.set_position(0, position)  # changes the position of servo 0
        if self.rudder is not None:
            angle = math.degrees(self.rudder)  # takes the rudder angle in radians and converts to degrees
            # k value is 6.301 because input is between -45 and 45 degrees
            position = int(self.rudder_k*(angle + self.rudder_mid)+992)
            # the slope between rudder and sail are different
            rospy.loginfo("Position Rudder:[%d]", position)  # logs the current position of the rudder
            self.mcl.set_position(1, position)  # changes the position of servo 1


if __name__ == '__main__':
    '''
    Initializes the node and subscribes to the two publishers. Continually listens as long as the node is running.
    '''
    rospy.init_node('motor_controller_listener')
    mcl = MotorControllerListener()
    queue_size = rospy.get_param("~/queue_size", 10)
    rospy.Subscriber('rudder_controller/rudder_angle', Float64, mcl.rudder_callback, queue_size=queue_size)
    rospy.Subscriber('sail_controller/sail_servo_angle', Float64, mcl.sail_callback, queue_size=queue_size)
    rospy.spin()
