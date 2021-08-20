#!/usr/bin/env python3
"""
receive the lane mask image and output a normalized point (x, y) to where the robot should go
Currently, the top most point is chosen, but other methods could replace this in the future

Example use:
    Subscribes to: yellow_mask
    msg info:
        sensor_msgs/Image

    Published to: yellow_centroid
    msg info:
        sensor_msgs/Image


Mohamed Martini
University of Massachusetts Lowell
"""

import rospy
import numpy as np
from sensor_msgs.msg import Image
from jetbot_msgs.msg import Vector2D


class NavigationPoint:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.main, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, Vector2D, queue_size=1)

    def get_nav_point(self, image):
        for y in range(image.shape[0]):
            for x in range(image.shape[1]):
                if image[y, x] > 0:
                    return (x, y)
        return NAV_POINT_DEF

    def publish(self, point):
        msg = Vector2D()
        msg.x, msg.y = point
        self.pub.publish(msg)

    def main(self, msg):
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)  # cv_bridge alternative
        nav_point = self.get_nav_point(image)
        self.publish(nav_point)


if __name__ == "__main__":
    NODE_NAME = "navigation_point_node"
    rospy.init_node(NODE_NAME, anonymous=True)

    IN_TOPIC = "in_topic"
    OUT_TOPIC = "out_topic"

    NAV_POINT_DEF = -1, -1

    NavigationPoint()
    rospy.spin()
