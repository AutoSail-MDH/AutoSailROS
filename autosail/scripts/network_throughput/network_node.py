#!/usr/bin/env python

import rospy
import os
import time
from std_msgs.msg import Float64
from network_throughput.network_script import network

def networkPublisher():
    net = network()
    txpub = rospy.Publisher('/network/tx_Mbps', Float64, queue_size=10)
    rxpub = rospy.Publisher('/network/rx_Mbps', Float64, queue_size=10)
    rospy.init_node('network_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        tx1 = net.get_data('tx')
        rx1 = net.get_data('rx')
        time.sleep(1)
        tx2 = net.get_data('tx')
        rx2 = net.get_data('rx')
        tx_speed = round((float(tx2) - float(tx1))/1000000.0, 4)
        rx_speed = round((float(rx2) - float(rx1))/1000000.0, 4)
        txpub.publish(tx_speed)
        rxpub.publish(rx_speed)
        rate.sleep()

if __name__ == '__main__':
    try:
        networkPublisher()
    except rospy.ROSInterruptException:
        pass