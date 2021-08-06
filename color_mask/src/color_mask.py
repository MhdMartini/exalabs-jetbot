#!/usr/bin/env python3
"""
Node to filter out and publish a thresholded yellow lane image

Subscribes to: camera/processed/cropped
msg info:
    sensor_msgs/Image

Published to: camera/yellow_mask
msg info:
    sensor_msgs/Image


Mohamed Martini
University of Massachusetts Lowell
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
import os


NODE_NAME = "color_mask_node"
IN_TOPIC = "in_topic"
OUT_TOPIC = "out_topic"

PARAM_HSV = os.path.join(rospy.get_name(), "HSV")
PARAM_HSV_DEF = {
    # yellow
    "LOWER": (1, 110, 150),
    "UPPER": (120, 255, 255)
}


class ColorMask:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.color_filter, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, Image, queue_size=1)

    def color_filter(self, msg):
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)  # cv_bridge alternative. Image is upside down
        mask = self.threshold_color(image)  # hsv filter -> mask
        self.publish(mask)

    def threshold_color(self, image):
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_vals = rospy.get_param(PARAM_HSV, PARAM_HSV_DEF)
        mask = cv2.inRange(image_hsv, np.array(hsv_vals["LOWER"]), np.array(hsv_vals["UPPER"]))
        return mask

    def publish(self, image):
        msg = Image()
        msg.data = bytes(image.astype(np.uint8))
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = "mono8"
        msg.step = len(msg.data) // msg.height
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)
    ColorMask()
    rospy.spin()
