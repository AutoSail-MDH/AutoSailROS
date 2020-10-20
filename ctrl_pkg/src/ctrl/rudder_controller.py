import math
import rospy
import std_msgs.msg
import threading
from simple_pid import PID


# def rudder_angle_calculation(pid_corrected_heading, velocity):
#     return rudder_angle

"""
def callback_desired_heading(data):
    desired_heading = rospy.loginfo(data.data)


def callback_course(data):
    course = rospy.loginfo(data.data)


def callback_heading(data):
    heading = rospy.loginfo(data.data)


def callback_velocity_flag(data):
    velocity_flag = rospy.loginfo(data.data)
"""

def pid_(desired_heading, course, heading, velocity_flag=False):
    kp = 1  # The Proportional term scalar
    ki = 0.1  # The Integral term scalar
    kd = 0.05  # The Derivative term scalar
    pid = PID(kp, ki, kd, 0)  # Initialization of the PID, with 0 as the control signal
    pid.output_limits = (-(math.pi/4), math.pi/4)  # Limits the rudder angles to [-45, 45] degrees
    """
    pub = rospy.Publisher("rudder_angle", std_msgs.msg.Float32, queue_size=1)
    r = rospy.Rate(10)  # Defines the publishing frequency to 10Hz
    rospy.Subscriber(name="pathplanner", data_class=std_msgs.msg.Float32, callback=callback_desired_heading,
                     queue_size=1)  # TODO: change the name to what is used by the pathplanner
    rospy.Subscriber(name="current_course", data_class=std_msgs.msg.Float32, callback=callback_course,
                     queue_size=1)  # TODO: change the name to what is used by the current course publishing
    rospy.Subscriber(name="current_heading", data_class=std_msgs.msg.Float32, callback=callback_heading,
                     queue_size=1)  # TODO: change the name of topic
    rospy.Subscriber(name="velocity_above_threshold", data_class=std_msgs.msg.Bool, callback=callback_velocity_flag,
                     queue_size=1)  # TODO: change the name
                     
    while None in {desired_heading, course, heading, velocity_flag}:
        continue
        
    while not rospy.is_shutdown():
        if velocity_flag:
            control_signal = course
        else:
            control_signal = heading
        pid.setpoint = desired_heading
        rudder_angle = pid(control_signal)
        pub.publish(rudder_angle)
        print(rudder_angle)
        # r.sleep()


#def trajectory_to_relative_heading(desired_trajectory, current_heading):
    #heading = desired_trajectory-current_heading
    #return heading

"""

class rudder_controller:

    def __init__(self):
        self.current_heading = None
        pid initialize

    def set_current_heading(self, heading):
        self.current_heading = heading

    def set_another_propertiy(self, nånting):
        self.nånting = nånting

    def output_heading(self):
        pid saker
        return ny course


if __name__ == "__main__":
    #pid_()

    rc = rudder_controller()
    rospy.Subscriber(name="pathplanner", data_class=std_msgs.msg.Float32, callback=rc.set_current_heading,
                     queue_size=1)

    rospy.init_node("pid")
    x = 0.1
    pub_test1 = rospy.Publisher("pathplanner", std_msgs.msg.Float32, queue_size=1)
    pub_test2 = rospy.Publisher("current_course", std_msgs.msg.Float32, queue_size=1)
    pub_test3 = rospy.Publisher("current_heading", std_msgs.msg.Float32, queue_size=1)
    pub_test4 = rospy.Publisher("velocity_above_threshold", std_msgs.msg.Bool, queue_size=1)

    # sett hertz


    #pid_thread = threading.Thread(target=pid_())
    #pid_thread.start()


    while not rospy.is_shutdown():
        nycourse = rc.update_course()
        publish stuff


    """while True:
        heading_test = math.sin(x)
        pub_test1.publish(heading_test)
        course_test = math.cos(x)
        pub_test2.publish(course_test)
        heading_now = math.cos(x-0.1)
        pub_test3.publish(heading_now)
        pub_test4.publish(True)
        x = (x+0.1) % 1
"""