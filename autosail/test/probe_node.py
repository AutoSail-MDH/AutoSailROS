#!/usr/bin/env python
import rospy
import threading
from utils.polls import poll_connections
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu, NavSatFix
from autosail.msg import stm32_msg
from rospy.exceptions import ROSException
import rostopic

class PublishTest:

    def __init__(self, topic: str, timeout: float):
        self.topic = topic
        self.timeout = timeout
        self.last_time = rospy.Time.now().secs

    def poll_publisher(self, topic_type):
        msg = None
        try:
            msg = rospy.wait_for_message(self.topic, topic_type, self.timeout)
        except ROSException as e:
            rospy.logerr(f"{self.topic} did not publish any msgs for {self.timeout}s")
        return msg

    def probe(self):
        rate = rospy.Rate(1)
        counter = 0
        topic_type = rostopic.get_topic_class(topic["name"])[0]
        while topic_type is None:
            if counter > 60 or counter == 0:
                rospy.logerr(f"Could not find topic {self.topic}")
                counter = 0
            topic_type = rostopic.get_topic_class(topic["name"])[0]
            rate.sleep()
            counter += 1
            if rospy.is_shutdown():
                return
        self.last_msg = topic_type()
        rate = rospy.Rate(0.5)
        rospy.loginfo(f"Started probin {self.topic}")
        while not rospy.is_shutdown():
            time = rospy.Time.now().secs
            msg = self.poll_publisher(topic_type)
            if hasattr(msg, 'header'):
                delattr(msg, 'header')
            if msg is not None and msg == self.last_msg:
                rospy.logwarn(f"{self.topic} did not change since last check, {time - self.last_time}s ago")
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("test_node", anonymous=True)

    topics = rospy.get_param("~topics")
    for topic in topics:
        pub_test = PublishTest(topic["name"], topic["timeout"])
        thr = threading.Thread(target=pub_test.probe)
        thr.start()
    rospy.spin()

    #wind_pub_test.test_connections(2)
    #imu_pub_test.test_connections(4)
    #fix_pub_test.test_connections(1)
    #fix_vel_pub_test.test_connections(3)

    #rudder_pub_test.test_connections(1)
    #sail_pub_test.test_connections(1)
