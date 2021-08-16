#!/usr/bin/env python3
"""
Node to estimate the slope inverse of a normalzied centroid and publish a normalized value (1 to -1)
If image is blank, 0 is published

Example use:
    Subscribes to: yellow_centroid
        msg info:
            jetbot_msgs/Vector2D

    Published to: yellow_slope
        msg info:
            std_msgs/Float32


Mohamed Martini
University of Massachusetts Lowell
"""

import rospy
from jetbot_msgs.msg import Vector2D
from std_msgs.msg import Float32
import numpy as np


class ObjectSlope:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Vector2D, self.main, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, Float32, queue_size=1)

    def get_dx_dy(self, x, y):
        # get pixel coordinates and return coordinates from bottom center
        # (x: 0.5 to -0.5 left to right) (y: 0 to 1 bottom to top)
        dx = 0.5 - x
        dy = 1 - y
        return dx, dy

    def publish(self, slope):
        self.pub.publish(slope)

    def main(self, msg):
        dx, dy = self.get_dx_dy(msg.x, msg.y)
        slope = np.arctan2(dx, dy)
        self.publish(slope)


if __name__ == "__main__":
    NODE_NAME = "object_slope_node"
    rospy.init_node(NODE_NAME, anonymous=True)

    IN_TOPIC = "in_topic"
    OUT_TOPIC = "out_topic"
    SLOPE_DEF = 0
    ObjectSlope()
    rospy.spin()
