#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg


def path_planner_partner_publisher():
    """
    A partner node for path_planner_node meant to simulate the nodes the path planner communicates with
    :return: Nothing
    """
    pub_waypoints = rospy.Publisher('path_planner/waypoints', std_msgs.msg.Float64MultiArray, queue_size=10)
    pub_gps_position = rospy.Publisher('gps/fix', sensor_msgs.msg.NavSatFix, queue_size=10)
    pub_gps_velocity = rospy.Publisher('gps/fix_velocity', geometry_msgs.msg.TwistWithCovarianceStamped, queue_size=10)
    pub_gps_heading = rospy.Publisher('/gps/navheading', sensor_msgs.msg.Imu, queue_size=10)
    pub_wind_sensor = rospy.Publisher('/wind_sensor', std_msgs.msg.Int64MultiArray, queue_size=10)
    pub_obstacles = rospy.Publisher('/path_planner/obstacles', std_msgs.msg.Float64MultiArray, queue_size=10)

    rospy.init_node('path_planner_partner')
    rate = rospy.Rate(60)  # 60hz
    # obstacles
    mat_obstacles = std_msgs.msg.Float64MultiArray()
    mat_obstacles.data = [59.606017, 16.568077, 59.606036, 16.567455, 59.605756, 16.567307]
    # waypoints
    mat_waypoints = std_msgs.msg.Float64MultiArray()
    mat_waypoints.data = [59.606502, 16.568896]  # 100, 100
    # gps position
    fix = sensor_msgs.msg.NavSatFix()
    fix.longitude = 16.567138
    fix.latitude = 59.605596
    # gps velocity 59.605596, 16.567138
    velocity = geometry_msgs.msg.TwistWithCovarianceStamped()
    velocity.twist.twist.linear.x = 2
    velocity.twist.twist.linear.y = 2
    # gps heading
    heading = sensor_msgs.msg.Imu()
    heading.orientation.x = 1
    heading.orientation.y = 2
    # wind sensor
    wind_data = std_msgs.msg.Int64MultiArray()
    wind_data.data = [15, 0]
    while not rospy.is_shutdown():
        # rospy.loginfo(mat.data)
        pub_waypoints.publish(mat_waypoints)
        pub_obstacles.publish(mat_obstacles)
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