#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import numpy as np
from marti_nav_msgs.msg import RoutePoint, Route
from autosail.msg import obstaclemsg
from autosail.msg import obstacles_array_msg
from scipy.spatial.transform import Rotation
from marti_common_msgs.msg import KeyValue


def path_planner_partner_publisher():
    """
    A partner node for path_planner_node meant to simulate the nodes the path planner communicates with
    :return: Nothing
    """
    pub_waypoints = rospy.Publisher('path_planner/waypoints', Route, queue_size=10)
    pub_gps_position = rospy.Publisher('gps/fix', sensor_msgs.msg.NavSatFix, queue_size=10)
    pub_gps_velocity = rospy.Publisher('gps/fix_velocity', geometry_msgs.msg.TwistWithCovarianceStamped, queue_size=10)
    pub_gps_heading = rospy.Publisher('/imu/data', sensor_msgs.msg.Imu, queue_size=10)
    pub_wind_sensor = rospy.Publisher('/wind_sensor/wind_vector', geometry_msgs.msg.Vector3Stamped, queue_size=10)
    pub_obstacles = rospy.Publisher('/path_planner/obstacles', obstacles_array_msg, queue_size=10)
    rospy.init_node('path_planner_partner', log_level=rospy.get_param("log_level", rospy.INFO))
    rate = rospy.Rate(60)  # 60hz
    # obstacles 59.619834, 16.557354
    mat_obstacle = obstaclemsg()
    mat_obstacle.latitude = 59.617369
    mat_obstacle.longitude = 16.560619
    obstacle_array = obstacles_array_msg()
    obstacle_array.data.append(mat_obstacle)
    mat_obstacle = obstaclemsg()
    mat_obstacle.latitude = 59.617540
    mat_obstacle.longitude = 16.561016
    obstacle_array.data.append(mat_obstacle)

    """
    mat_waypoint = waypointmsg()
    mat_waypoint.latitude = 59.617829
    mat_waypoint.longitude = 16.560237
    mat_waypoint.id = 1
    waypoint_array = waypoint_array_msg()
    waypoint_array.data.append(mat_waypoint)
    mat_waypoint = waypointmsg()
    mat_waypoint.latitude = 59.618133
    mat_waypoint.longitude = 16.561035
    mat_waypoint.id = 0
    waypoint_array.data.append(mat_waypoint)
    """
    #
    waypoint_array = Route()
    waypoint = RoutePoint() # 90 deg 59.617494040283596, 16.56043112576914
    waypoint.pose.position.x = 16.56043112576914
    waypoint.pose.position.y = 59.617494040283596
    waypoint.id = "1"
    prop = KeyValue()
    prop.key = "diameter"
    prop.value = "5"
    waypoint.properties.append(prop)
    id = KeyValue()
    prop.key = "id"
    prop.value = "0"
    waypoint.properties.append(id)
    waypoint_array.route_points.append(waypoint)
    waypoint = RoutePoint() # 45deg 59.617907031489985, 16.56098932361882
    waypoint.pose.position.x = 16.56098932361882
    waypoint.pose.position.y = 59.617907031489985
    waypoint.id = "0"
    prop = KeyValue()
    prop.key = "diameter"
    prop.value = "5"
    waypoint.properties.append(prop)
    id = KeyValue()
    prop.key = "id"
    prop.value = "0"
    waypoint.properties.append(id)
    waypoint_array.route_points.append(waypoint)

    # gps position
    fix = sensor_msgs.msg.NavSatFix()
    # 59.617465001648355, 16.560823459114914
    fix.longitude = 16.560823459114914
    fix.latitude = 59.617465001648355

    # gps velocity
    velocity = geometry_msgs.msg.TwistWithCovarianceStamped()
    velocity.twist.twist.linear.x = 1
    velocity.twist.twist.linear.y = 0
    # imu heading
    heading = sensor_msgs.msg.Imu()
    rot = Rotation.from_euler('xyz', [0, 0, 45], degrees=True)
    rot_quat = rot.as_quat()
    heading.orientation.x = rot_quat[0]
    heading.orientation.y = rot_quat[1]
    heading.orientation.z = rot_quat[2]
    heading.orientation.w = rot_quat[3]

    wind_data = geometry_msgs.msg.Vector3Stamped()
    wind_data.vector.x = 7
    wind_data.vector.y = 7
    while not rospy.is_shutdown():
        pub_waypoints.publish(waypoint_array)
        pub_obstacles.publish(obstacle_array)
        pub_gps_position.publish(fix)
        pub_gps_velocity.publish(velocity)
        pub_gps_heading.publish(heading)
        pub_wind_sensor.publish(wind_data)
        rate.sleep()


if __name__ == '__main__':
    try:
        path_planner_partner_publisher()
    except rospy.ROSInterruptException:
        pass
