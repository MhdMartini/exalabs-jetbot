#!/usr/bin/env python3
"""
receive the lane mask image and output a normalized point (x, y) to where the robot should go
Navigation point is outputted from a trained neural network

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
from navigation_point import NavigationPoint


class DLPoint(NavigationPoint):
    def __init__(self):
        super(NavigationPoint, self).__init__()

    def get_nav_point(self, image, height, width):
        return NAV_POINT_DEF


if __name__ == "__main__":
    NODE_NAME = "navigation_point_dl_node"
    rospy.init_node(NODE_NAME, anonymous=True)

    IN_TOPIC = "in_topic"
    OUT_TOPIC = "out_topic"

    NAV_POINT_DEF = -1, -1

    DLPoint()
    rospy.spin()
