#!/usr/bin/env python
import time
from rospy import Subscriber, Publisher, Rate
from typing import Union


def poll_connections(pub_sub: Union[Publisher, Subscriber], required_conns: int, timeout: float) -> int:
    """
    Polls connections for a Publisher or Subscriber and returns true if the required connections are achieved
    :param pub_sub: Publisher or Subscriber
    :param required_conns: Required connections to wait for
    :param timeout: The amount of seconds to poll
    :return: The amount of connections
    """
    poll_rate = Rate(100)
    timeout = time.time() + timeout
    connections = pub_sub.get_num_connections()
    while connections < required_conns and time.time() < timeout:
        connections = pub_sub.get_num_connections()
        poll_rate.sleep()
    return connections

