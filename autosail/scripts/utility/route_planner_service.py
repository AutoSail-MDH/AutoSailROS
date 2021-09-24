#!/usr/bin/env python
import rospy
from marti_nav_msgs.srv import PlanRoute, PlanRouteResponse
from marti_nav_msgs.msg import RoutePoint
from marti_common_msgs.msg import KeyValue


def handle_route_planner(req):
    res = PlanRouteResponse()
    id_c = 0
    for wp in req.waypoints:
        point = RoutePoint()
        point.pose = wp
        point.id = str(id_c)
        point.pose.orientation.w = 1
        prop = KeyValue()
        prop.key = "diameter"
        prop.value = "5"
        point.properties.append(prop)
        id = KeyValue()
        prop.key = "id"
        prop.value = "0"
        point.properties.append(id)
        res.route.route_points.append(point)
        id_c += 1
    res.route.header.stamp = rospy.Time.now()
    res.success = True
    return res


if __name__ == "__main__":
    rospy.init_node('testing_service', log_level=rospy.get_param("log_level", rospy.INFO))
    rospy.Service('plan_route', PlanRoute, handle_route_planner)
    rospy.spin()

