rospy.init_node("pid")
"""
def callback_desired_heading(data):
    desired_heading = rospy.loginfo(data.data)


def callback_course(data):
    course = rospy.loginfo(data.data)


def callback_heading(data):
    heading = rospy.loginfo(data.data)


def callback_velocity_flag(data):
    velocity_flag = rospy.loginfo(data.data)
    
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
"""

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