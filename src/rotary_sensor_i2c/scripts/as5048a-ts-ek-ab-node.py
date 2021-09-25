import rospy
import spidev
from std_msgs.msg import Float32


def read_angle():
    pub = rospy.Publisher('angle', Float32, queue_size=10)
    rospy.init_node(name='as5048a-ts-ek-ab', anonymous=False)
    rate = rospy.Rate(10) # 10Hz

    # Initialize SPI
    spi = spidev.SpiDev()
    spi.open(0, 0)
    # Settings
    spi.max_speed_hz = 5000
    spi.mode = 0b01



    while not rospy.is_shutdown():
        data = spi.readbytes(2) # 2 bytes data
        rate.sleep()
    spi.close() # Close when shutdown


if __name__ == '__main__':
    try:
        read_angle()
    except rospy.ROSInterruptException:
        pass # TODO: Handle exception properly