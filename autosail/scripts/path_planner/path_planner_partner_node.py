#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import numpy as np
from marti_nav_msgs.msg import RoutePoint, Route
from autosail.msg import waypointmsg
from autosail.msg import waypoint_array_msg
from autosail.msg import obstaclemsg
from autosail.msg import obstacles_array_msg


def path_planner_partner_publisher():
    """
    A partner node for path_planner_node meant to simulate the nodes the path planner communicates with
    :return: Nothing
    """
    pub_waypoints = rospy.Publisher('path_planner/waypoints', Route, queue_size=10)
    pub_gps_position = rospy.Publisher('gps/fix', sensor_msgs.msg.NavSatFix, queue_size=10)
    pub_gps_velocity = rospy.Publisher('gps/fix_velocity', geometry_msgs.msg.TwistWithCovarianceStamped, queue_size=10)
    pub_gps_heading = rospy.Publisher('/imu/data', sensor_msgs.msg.Imu, queue_size=10)
    pub_wind_sensor = rospy.Publisher('/wind_sensor', geometry_msgs.msg.Vector3Stamped, queue_size=10)
    pub_obstacles = rospy.Publisher('/path_planner/obstacles', obstacles_array_msg, queue_size=10)
    rospy.init_node('path_planner_partner')
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
    mat_obstacle = obstaclemsg()
    mat_obstacle.latitude = 59.617752
    mat_obstacle.longitude = 16.560581
    obstacle_array.data.append(mat_obstacle)
    mat_obstacle = obstaclemsg()
    mat_obstacle.latitude = 59.617584
    mat_obstacle.longitude = 16.560828
    obstacle_array.data.append(mat_obstacle)
    mat_obstacle = obstaclemsg()
    mat_obstacle.latitude = 59.617410
    mat_obstacle.longitude = 16.560780
    obstacle_array.data.append(mat_obstacle)
    mat_obstacle = obstaclemsg()
    mat_obstacle.latitude = 59.617318
    mat_obstacle.longitude = 16.560630
    obstacle_array.data.append(mat_obstacle)
    mat_obstacle = obstaclemsg()
    mat_obstacle.latitude = 59.617313
    mat_obstacle.longitude = 16.560759
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
    waypoint_array = Route()
    waypoint = RoutePoint()
    waypoint.pose.position.x = 2.928389
    waypoint.pose.position.y = 57.248095
    waypoint.id = "0"
    waypoint_array.route_points.append(waypoint)
    waypoint = RoutePoint()
    waypoint.pose.position.x = 2.930238
    waypoint.pose.position.y = 57.246999
    waypoint.id = "0"
    waypoint_array.route_points.append(waypoint)

    # gps position
    fix = sensor_msgs.msg.NavSatFix()
    fix.latitude = 57.248346
    fix.longitude = 2.927921
    # gps velocity
    velocity = geometry_msgs.msg.TwistWithCovarianceStamped()
    velocity.twist.twist.linear.x = 0
    velocity.twist.twist.linear.y = 0
    # gps heading
    heading = sensor_msgs.msg.Imu()
    heading.orientation.x = 1
    heading.orientation.y = 1
    # wind sensor geometry_msgs.msg.Vector3Stamped
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
