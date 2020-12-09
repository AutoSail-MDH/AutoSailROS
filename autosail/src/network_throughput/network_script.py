#!/usr/bin/env python
import time


class network:
    def __init__(self):
        pass

    def get_rx(self):
        with open('/sys/class/net/eno1/statistics/rx_bytes', 'r') as f:
            data_read = f.read()
        return data_read

    def get_tx(self):
        with open('/sys/class/net/eno1/statistics/tx_bytes', 'r') as f:
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
