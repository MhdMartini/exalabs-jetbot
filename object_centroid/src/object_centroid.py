#!/usr/bin/env python3
"""
Node to receive a binary mask and publish the normalized centroid of that mask. If mask is empty, centroid is -1, -1

Example use:
    Subscribes to: camera/color_mask
        msg info:
            sensor_msgs/Image

    Published to: yellow_lane_centroid
        msg info:
            jetbot_msgs/Vector2D
                float32     x
                float32     y


Mohamed Martini
University of Massachusetts Lowell
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from jetbot_msgs.msg import Vector2D


class ObjectCentroid:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.main, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, Vector2D, queue_size=1)

    def find_centroid(self, mask):
        # find the relative horizontal angle of the yellow lane [1 -> -1]
        moments = cv2.moments(mask)
        try:
            center_x = int(moments["m10"] / moments["m00"])
            center_y = int(moments["m01"] / moments["m00"])
            return center_x, center_y
        except ZeroDivisionError:  # no lane is detected
            return None

    def normalize(self, height, width, point):
        col, row = point
        return (col / width, row / height)

    def publish(self, centroid):
        msg = Vector2D()
        msg.x, msg.y = centroid
        self.pub.publish(msg)

    def main(self, msg):
        mask = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)  # cv_bridge alternative
        centroid = self.find_centroid(mask)
        if centroid is None:
            self.publish(CENTROID_DEF)
            return
        centroid_norm = self.normalize(msg.height, msg.width, centroid)
        self.publish(centroid_norm)


if __name__ == "__main__":

    NODE_NAME = "object_centroid_node"
    rospy.init_node(NODE_NAME, anonymous=True)

    IN_TOPIC = "in_topic"
    OUT_TOPIC = "out_topic"
    CENTROID_DEF = (-1, -1)

    ObjectCentroid()
    rospy.spin()
