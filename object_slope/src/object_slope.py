#!/usr/bin/env python3
"""
Node to estimate the slope inverse of a binary mask and publish a normalized value (1 to -1)
If image is blank, slope is 0

Example use:
    Subscribes to: yellow_mask
        msg info:
            sensor_msgs/Image

    Published to: lane_slope
        msg info:
            std_msgs/Float32


Mohamed Martini
University of Massachusetts Lowell
"""

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import numpy as np


class ObjectSlope:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.main, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, Float32, queue_size=1)

    def get_points(self, image):
        points = np.any(image == 255)
        top_point_row = points[0][0]
        top_point_col = points[1][0]
        bottom_point_row = points[0][-1]
        bottom_point_col = points[1][-1]
        return (top_point_row, top_point_col), (bottom_point_row, bottom_point_col)

    def find_op_adj(self, points):
        top, bottom = points
        opp = bottom[0] - top[0]
        adj = bottom[1] - top[1]
        return opp, adj

    def find_angle(self, opposite, adjacent):
        try:
            rad = np.arctan(opposite / adjacent)
        except ZeroDivisionError:
            rad = np.pi / 2
        return rad

    def publish(self, slope_norm):
        self.pub.publish(slope_norm)

    def main(self, msg):
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)  # cv_bridge alternative
        try:
            points = self.get_points(image)
        except IndexError:
            self.publish(SLOPE_DEF)
            return

        opp, adj = self.find_op_adj(points)
        angle = self.find_angle(opp, adj)
        slope_norm = np.sign(angle) * np.cos(angle)
        self.publish(slope_norm)


if __name__ == "__main__":
    NODE_NAME = "object_slope_node"
    rospy.init_node(NODE_NAME, anonymous=True)

    IN_TOPIC = "in_topic"
    OUT_TOPIC = "out_topic"
    SLOPE_DEF = 0
    ObjectSlope()
    rospy.spin()
