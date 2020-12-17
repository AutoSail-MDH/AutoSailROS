#!/usr/bin/env python
import time
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

    def __init__(self, topic: str):
        self.topic = topic
        self.last_time = rospy.Time.now().secs
        self.msg_received = 0
        self.subscribed = 0
        self.msg = None
        self.last_msg = None


    def subscribe(self):
        if self.subscribed:
            return
        topic_type = rostopic.get_topic_class(self.topic)[0]
        if topic_type is None:
            rospy.logerr(f"Could not find topic {self.topic}")
            return
        rospy.Subscriber(self.topic, topic_type, self.callback, queue_size=10)
        rospy.loginfo(f"Started probin {self.topic}")
        self.subscribed = 1

    def callback(self, msg):
        self.msg = msg
        self.msg_received = 1


if __name__ == "__main__":
    rospy.init_node("test_node", anonymous=True)

    topics = rospy.get_param("~topics")
    rospy.logerr(topics)
    pub_tests = []
    for topic in topics:
        pub_tests.append(PublishTest(topic["name"]))

    rate = rospy.Rate(0.5)
    last_time = time.time()
    while not rospy.is_shutdown():
        time_now = time.time()
        for pub_test in pub_tests:
            pub_test.subscribe()
            if not pub_test.msg_received:
                rospy.logerr(f"{pub_test.topic} did not publish any msgs for {time_now - last_time}s")
            else:
                msg = pub_test.msg
                if hasattr(msg, 'header'):
                    delattr(msg, 'header')
                if pub_test is not None and msg == pub_test.last_msg:
                    rospy.logwarn(f"{pub_test.topic} did not change since last check, {time_now - last_time}s ago")
            pub_test.last_msg = pub_test.msg
            pub_test.msg_received = 0
        last_time = time_now
        rate.sleep()
