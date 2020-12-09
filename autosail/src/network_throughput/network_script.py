#!/usr/bin/env python
import time
import os

class network:
    def __init__(self):
        pass

    def get_data(self, var):
        if os.uname()[4][0:3] == 'x86':
            with open('/sys/class/net/eno1/statistics/' + var + '_bytes', 'r') as f:
                data_read = f.read()
        else:
            with open('/sys/class/net/eth0/statistics/' + var + '_bytes', 'r') as f:
                data_read = f.read()
        return data_read

if __name__ == "__main__":
    while True:
        tx1 = network.get_tx(network)
        rx1 = network.get_rx(network)
        time.sleep(1)
        tx2 = network.get_tx(network)
        rx2 = network.get_rx(network)
        tx_speed = round((float(tx2) - float(tx1))/1000000.0, 4)
        rx_speed = round((float(rx2) - float(rx1))/1000000.0, 4)
        print(f"TX: {tx_speed}Mbps, RX: {rx_speed}Mbps")
