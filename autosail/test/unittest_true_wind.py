import math
import numpy as np
from unittest import TestCase
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from marti_nav_msgs.msg import Route, RoutePoint
import rosunit


class TestTrueWind(TestCase):

    def output_callback(self, msg):
        self.output = [msg.vector.x, msg.vector.y]

    def setUp(self):
        rospy.init_node("test_true_wind")
        self.pub_gps_velocity = rospy.Publisher('gps/fix_velocity', TwistWithCovarianceStamped, queue_size=10)
        self.pub_gps_heading = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.pub_wind_sensor = rospy.Publisher('/wind_sensor', Vector3Stamped, queue_size=10)
        rospy.Subscriber('/wind_sensor/true', Vector3Stamped, self.output_callback, queue_size=1)
        self.output = None
        self.rate = rospy.Rate(1)

    def tearDown(self):
        rospy.signal_shutdown("Test ended")

    def test_input_output(self):
        # gps velocity
        velocity = TwistWithCovarianceStamped()
        velocity.twist.twist.linear.x = -1
        velocity.twist.twist.linear.y = 0
        # gps heading
        heading = Imu()
        heading.orientation.x = 0
        heading.orientation.y = 0
        heading.orientation.z = 0
        heading.orientation.w = 1
        # wind sensor geometry_msgs.msg.Vector3Stamped
        wind_data = Vector3Stamped()
        wind_data.vector.x = 1
        wind_data.vector.y = 1
        while self.output == None and not rospy.is_shutdown():
            self.pub_gps_heading.publish(heading)
            self.pub_wind_sensor.publish(wind_data)
            self.pub_gps_velocity.publish(velocity)
            self.rate.sleep()

        self.assertEqual(self.output, [0, 1])




if __name__ == "__main__":
    rosunit.unitrun("autosail", "true_wind", TestTrueWind)