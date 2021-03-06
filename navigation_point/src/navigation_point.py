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

    def get_nav_point(self, image, height, width):
        """
        Get the top-most filtered point
        """
        NAV_POINT_DEF = 0, 0
        for y in range(height):
            if not np.any(image[y]):
                continue
            center_x = int(np.mean(np.where(image[y] > 0)[0]))
            return center_x, y
        return NAV_POINT_DEF

    def publish(self, point):
        msg = Vector2D()
        msg.x, msg.y = point
        self.pub.publish(msg)

    def adjust_point(self, point, height, width):
        """
        receive a point coordinates in numpy array and transform them to the jetbot's perspective
        """
        np_x, np_y = point
        x = np_x - width // 2
        y = height - np_y
        return x, y

    def normalize_point(self, point, height, width):
        col, row = point
        return (col / width, row / height)

    def main(self, msg):
        height, width = msg.height, msg.width
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, -1)  # cv_bridge alternative
        nav_point = self.get_nav_point(image, height, width)
        nav_point_adj = self.adjust_point(nav_point, height, width)
        nav_point_norm = self.normalize_point(nav_point_adj, height, width)
        self.publish(nav_point_norm)


if __name__ == "__main__":
    NODE_NAME = "navigation_point_node"
    rospy.init_node(NODE_NAME, anonymous=True)

    IN_TOPIC = "in_topic"
    OUT_TOPIC = "out_topic"

    NAV_POINT_DEF = -1, -1

    NavigationPoint()
    rospy.spin()
