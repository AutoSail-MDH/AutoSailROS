import rospy
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import numpy as np
from marti_nav_msgs.msg import RoutePoint, Route
from autosail.msg import obstaclemsg
from autosail.msg import obstacles_array_msg
from scipy.spatial.transform import Rotation
import std_msgs.msg

desired_course = None
rudder_angle = None

class FakeSignals:
    def __init__(self):
        # --------- path planner sensors -----
        # waypoints
        waypoint_array = Route()
        waypoint = RoutePoint()  # 90 deg
        waypoint.pose.position.x = 16.56172953630712
        waypoint.pose.position.y = 59.617444802123934
        waypoint.id = "0"
        self.waypoints = waypoint_array.route_points.append(waypoint)
        # obstacle
        mat_obstacle = obstaclemsg()
        mat_obstacle.latitude = 59.617369
        mat_obstacle.longitude = 16.560619
        obstacle_array = obstacles_array_msg()
        obstacle_array.data.append(mat_obstacle)
        self.obstacle = obstacle_array
        # wind sensor
        wind_sensor_value = Vector3Stamped()
        wind_sensor_value.vector.x = 7
        wind_sensor_value.vector.y = 7
        wind_sensor_value.vector.z = 0
        self.wind_sensor = wind_sensor_value
        # imu heading
        heading = sensor_msgs.msg.Imu()
        rot = Rotation.from_euler('xyz', [0, 0, 90], degrees=True)
        rot_quat = rot.as_quat()
        heading.orientation.x = rot_quat[0]
        heading.orientation.y = rot_quat[1]
        heading.orientation.z = rot_quat[2]
        heading.orientation.w = rot_quat[3]
        self.heading = heading
        # gps velocity
        gps_velocity_value = TwistWithCovarianceStamped()
        gps_velocity_value.twist.twist.linear.x = 1
        gps_velocity_value.twist.twist.linear.y = 0
        gps_velocity_value.twist.twist.linear.z = 0
        self.gps_velocity = gps_velocity_value
        # gps position
        gps_position_value = sensor_msgs.msg.NavSatFix()
        gps_position_value.longitude = 16.560831863216134
        gps_position_value.latitude = 59.61745620958708
        self.gps_position = gps_position_value
        # -----------------------------------

class Publisher:
    def __init__(self):
        self.pub_waypoints = rospy.Publisher('path_planner/waypoints', Route, queue_size=10)
        self.pub_obstacle = rospy.Publisher('/path_planner/obstacles', obstacles_array_msg, queue_size=10)
        self.pub_wind_sensor = rospy.Publisher('/wind_sensor', geometry_msgs.msg.Vector3Stamped, queue_size=10)
        self.pub_imu = rospy.Publisher('/imu/data', sensor_msgs.msg.Imu, queue_size=10)
        self.pub_velocity = rospy.Publisher('gps/fix_velocity', geometry_msgs.msg.TwistWithCovarianceStamped, queue_size=10)
        self.pub_position = rospy.Publisher('gps/fix', sensor_msgs.msg.NavSatFix, queue_size=10)

class TestValues:
    def __init__(self):
        self.desired_course = np.pi/2
        self.rudder_angle = np.pi/4

def callback_desired_course(data):
    global desired_course
    desired_course = data.data

def callback_rudder_angle(data):
    global rudder_angle
    rudder_angle = data.data

def callback_sail_angle(data):
    global sail_angle

def init_subsriber():
    rospy.Subscriber(name="/path_planner/course", data_class=std_msgs.msg.Float64,
                     callback=callback_desired_course, queue_size=1)
    rospy.Subscriber(name="/rudder_controller/rudder_angle", data_class=std_msgs.msg.Float64,
                     callback=callback_rudder_angle, queue_size=1)

def publish_signals(fake_signals, publisher):
    publisher.pub_waypoints.publish(fake_signals.waypoints)
    publisher.pub_obstacle.publish(fake_signals.obstacle)
    publisher.pub_wind_sensor.publish(fake_signals.wind_sensor)
    publisher.pub_imu.publish(fake_signals.heading)
    publisher.pub_velocity.publish(fake_signals.gps_velocity)
    publisher.pub_position.publish(fake_signals.gps_position)

def test_system():
    global desired_course, rudder_angle
    init_subsriber()
    fake_signals = FakeSignals
    publisher = Publisher
    test_values = TestValues

    while not rospy.is_shutdown() and desired_course is None and rudder_angle is None:
        publish_signals(fake_signals=fake_signals, publisher=publisher)

    if abs(test_values.desired_course - desired_course) < np.deg2rad(5):
        print("Startup test-Path planner: Succeeded")
    else:
        print("Startup test-Path planner: Failed")
    if abs(test_values.rudder_angle - rudder_angle) < np.deg2rad(5):
        print("Startup test-Rudder control: Succeeded")
    else:
        print("Startup test-Rudder control: Failed")


        # - pi/4


