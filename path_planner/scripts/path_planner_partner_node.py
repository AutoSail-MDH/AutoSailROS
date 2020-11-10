#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
from path_planner.msg import waypointmsg
from path_planner.msg import waypoint_array_msg
from path_planner.msg import obstaclemsg
from path_planner.msg import obstacles_array_msg


def path_planner_partner_publisher():
    """
    A partner node for path_planner_node meant to simulate the nodes the path planner communicates with
    :return: Nothing
    """
    pub_waypoints = rospy.Publisher('path_planner/waypoints', waypoint_array_msg, queue_size=10)
    pub_gps_position = rospy.Publisher('gps/fix', sensor_msgs.msg.NavSatFix, queue_size=10)
    pub_gps_velocity = rospy.Publisher('gps/fix_velocity', geometry_msgs.msg.TwistWithCovarianceStamped, queue_size=10)
    pub_gps_heading = rospy.Publisher('/imu/data', sensor_msgs.msg.Imu, queue_size=10)
    pub_wind_sensor = rospy.Publisher('/wind_sensor', std_msgs.msg.Float64MultiArray, queue_size=10)
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
    mat_waypoint = waypointmsg()
    # 59.617829, 16.560237
    mat_waypoint.latitude = 59.617829
    mat_waypoint.longitude = 16.560237
    mat_waypoint.id = 0
    waypoint_array = waypoint_array_msg()
    waypoint_array.data.append(mat_waypoint)
    mat_waypoint = waypointmsg()
    mat_waypoint.latitude = 59.618133
    mat_waypoint.longitude = 16.561035
    mat_waypoint.id = 1
    waypoint_array.data.append(mat_waypoint)
    # gps position 59.617595, 16.561043 59.617459, 16.560839IndexError: list assignment index out of range

    fix = sensor_msgs.msg.NavSatFix()
    fix.latitude = 59.617459
    fix.longitude = 16.560839
    # gps velocity
    velocity = geometry_msgs.msg.TwistWithCovarianceStamped()
    velocity.twist.twist.linear.x = 2
    velocity.twist.twist.linear.y = 2
    # gps heading
    heading = sensor_msgs.msg.Imu()
    heading.orientation.x = 1
    heading.orientation.y = 2
    # wind sensor
    wind_data = std_msgs.msg.Float64MultiArray()
    wind_data.data = [20, 3.142]  # 5.498 0.7854
    while not rospy.is_shutdown():
        pub_waypoints.publish(waypoint_array)
        # pub_obstacles.publish(obstacle_array)
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
