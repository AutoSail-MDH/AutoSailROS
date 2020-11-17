import rospy
from std_msgs.msg import Float64, Float64MultiArray, Int64MultiArray
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from sensor_msgs.msg import Imu, NavSatFix


class FakeSignals:
    def __init__(self):
        gps_position_value = NavSatFix()
        gps_position_value.latitude = 0
        gps_position_value.longitude = 0
        self.gps_position = gps_position_value

        gps_velocity_value = TwistWithCovarianceStamped()
        gps_velocity_value.twist.twist.linear.x = 0
        gps_velocity_value.twist.twist.linear.y = 0
        gps_velocity_value.twist.twist.linear.z = 0
        self.gps_velocity = gps_velocity_value

        wind_sensor_value = Vector3Stamped()
        wind_sensor_value.vector.x = 0
        wind_sensor_value.vector.y = 0
        wind_sensor_value.vector.z = 0
        self.wind_sensor = wind_sensor_value

        imu_value = Imu()
        imu_value.linear_acceleration = 0
        imu_value.angular_velocity = 0
        self.imu = imu_value

        water_level_value = Float64MultiArray()
        water_level_value.layout.dim = 1
        water_level_value.data = [0., 0.]
        self.water_level = water_level_value

        water_detect_value = Int64MultiArray()
        water_detect_value.layout.dim = 1
        water_detect_value.data = [0, 0]
        self.water_detect = water_detect_value

        current_sensor_value = Float64MultiArray()
        current_sensor_value.layout.dim = 1
        current_sensor_value.data = [0., 0., 0., 0., 0.]
        self.current_sensor = current_sensor_value

        camera_values = Float64MultiArray()
        camera_values.layout.dim = 1
        camera_values.data = [0., 0.]
        self.camera = camera_values


def publish_signals(values):
    global gps_pos_pub
    global gps_velocity_pub
    global wind_pub
    global imu_pub
    # --- TODO: uncomment when fixed
    # global water_level_pub
    # global water_detect_pub
    # global current_pub
    # global camera_pub
    # -----

    gps_pos_pub.publish(values.gps_position)
    gps_velocity_pub.publish(values.gps_velocity)
    wind_pub.publish(values.wind_sensor)
    imu_pub.publish(values.imu)
    # --- TODO: uncomment when fixed
    # water_level_pub.publish(values.water_level)
    # water_detect_pub.publish(values.water_detect)
    # current_pub.publish(values.current_sensor)
    # camera_pub.publish(values.camera)
    # -----


rudder_angle = 0
sail_servo_angle = 0


def callback_rudder(data):
    global rudder_angle
    rudder_angle = data.data


def callback_sail(data):
    global sail_servo_angle
    sail_servo_angle = data.data


if __name__ == "__main__":
    sensor_values = FakeSignals()

    gps_pos_pub = rospy.Publisher(name="gps/navheading", data_class=NavSatFix, queue_size=1)
    gps_velocity_pub = rospy.Publisher(name="gps/fix_velocity", data_class=TwistWithCovarianceStamped, queue_size=1)
    wind_pub = rospy.Publisher(name="wind/apparent_rad", data_class=Vector3Stamped, queue_size=1)
    imu_pub = rospy.Publisher(name="imu/data", data_class=Imu, queue_size=1)
    # ---- TODO: fix
    # water_level_pub = rospy.Publisher(name="")
    # water_detect_pub = rospy.Publisher(name="")
    # current_pub = rospy.Publisher(name="")
    # camera_pub = rospy.Publisher(name="")
    # -----

    rospy.Subscriber(name="/rudder_controller/rudder_angle", data_class=Float64, callback=callback_rudder, queue_size=1)
    rospy.Subscriber(name="sail_controller/sail_servo_angle", data_class=Float64, callback=callback_sail, queue_size=1)

    rate = rospy.Rate(1)
    for i in range(10):
        publish_signals(sensor_values)
        rate.sleep()





