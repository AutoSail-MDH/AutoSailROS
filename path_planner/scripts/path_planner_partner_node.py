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
    # pub_gps_heading = rospy.Publisher('/filter/quaternion', geometry_msgs.msg.QuaternionStamped, queue_size=10)
    pub_wind_sensor = rospy.Publisher('/wind_sensor', std_msgs.msg.Float64MultiArray, queue_size=10)
    pub_obstacles = rospy.Publisher('/path_planner/obstacles', std_msgs.msg.Float64MultiArray, queue_size=10)
    rospy.init_node('path_planner_partner')
    rate = rospy.Rate(60)  # 60hz
    # obstacles 59.619834, 16.557354
    mat_obstacles = std_msgs.msg.Float64MultiArray()
    mat_obstacles.data = [59.617369, 16.560619, 59.617540, 16.561016, 59.617752, 16.560581, 59.617584, 16.560828,
                          59.617410, 16.560780, 59.617318, 16.560630, 59.617313, 16.560759]
    # waypoints [59.620879, 16.562914, 59.616884, 16.565244, 59.614469, 16.560352, 59.617795, 16.556490]
    mat_waypoints = std_msgs.msg.Float64MultiArray()
#    mat_waypoints.data = [59.620879, 16.562914, 59.616884, 16.565244, 59.614469, 16.560352, 59.617795, 16.556490]
    mat_waypoints.data = [59.621189, 16.555377, 59.616463, 16.560807, 59.617339, 16.555198, 59.622185, 16.567209,
                          59.635126, 16.552199, 59.617628, 16.560641]
    # gps position 59.617595, 16.561043 59.617459, 16.560839
    fix = sensor_msgs.msg.NavSatFix()
    fix.latitude = 59.617459
    fix.longitude = 16.560839
    # gps velocity
    velocity = geometry_msgs.msg.TwistWithCovarianceStamped()
    velocity.twist.twist.linear.x = 2
    velocity.twist.twist.linear.y = 2
    # gps heading
    heading = geometry_msgs.msg.QuaternionStamped()
    heading.quaternion.x = 1
    heading.quaternion.y = 2
    # wind sensor
    wind_data = std_msgs.msg.Float64MultiArray()
    wind_data.data = [20, 3.142]  # 5.498 0.7854
    while not rospy.is_shutdown():
        pub_waypoints.publish(mat_waypoints)
        pub_obstacles.publish(mat_obstacles)
        pub_gps_position.publish(fix)
        pub_gps_velocity.publish(velocity)
        # pub_gps_heading.publish(heading)
        pub_wind_sensor.publish(wind_data)
        rate.sleep()


if __name__ == '__main__':
    try:
        path_planner_partner_publisher()
    except rospy.ROSInterruptException:
        pass
