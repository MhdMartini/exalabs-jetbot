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
import torchvision
import torch


class NavigationPoint:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.main, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, Vector2D, queue_size=1)

    def get_nav_point(self, image, height, width):
        for y in range(height):
            for x in range(width):
                if image[y, x] > 0:
                    return x / width, y / height
        return NAV_POINT_DEF

    def publish(self, point):
        msg = Vector2D()
        msg.x, msg.y = point
        self.pub.publish(msg)

    def main(self, msg):
        height, width = msg.height, msg.width
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, -1)  # cv_bridge alternative
        nav_point = self.get_nav_point(image, height, width)
        self.publish(nav_point)


if __name__ == "__main__":
    NODE_NAME = "navigation_point_dl_node"
    rospy.init_node(NODE_NAME, anonymous=True)

    IN_TOPIC = "in_topic"
    OUT_TOPIC = "out_topic"

    NAV_POINT_DEF = -1, -1

    model = torchvision.models.resnet18(pretrained=False)
    model.fc = torch.nn.Linear(512, 2)

    NavigationPoint()
    rospy.spin()
