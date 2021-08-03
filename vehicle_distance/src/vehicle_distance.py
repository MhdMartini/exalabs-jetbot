#!/usr/bin/env python3
"""
Node to publish distance to the jetbot in front.

Subscribes to: camera/processed

Publishes to: vehicle_distance
msg info:
    Float32
        data    Float32


Mohamed Martini
University of Massachusetts Lowell
"""
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image


NODE_NAME = "vehicle_distance_node"
IN_TOPIC = "camera/processed"
OUT_TOPIC = "vehicle_distance"


class VehicleDistance:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.calc_distance, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, Float32, queue_size=1)

    def calc_distance(self, msg):
        # later replace with aruco tag distance
        distance = 1
        self.publish(distance)

    def publish(self, distance):
        self.pub.publish(distance)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    VehicleDistance()
    rospy.spin()
