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


NODE_NAME = "yellow_mask_node"
IN_TOPIC = "camera/processed/cropped"
OUT_TOPIC = "camera/yellow_mask"

YELLOW = {
    # hsv range of yellow color. IMPORTANT: Change for different lighting conditions
    "lower": (25, 142, 110),
    "upper": (39, 255, 207)
}


class YellowMask:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.yellow_filter, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, Image, queue_size=1)

    def yellow_filter(self, msg):
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)  # cv_bridge alternative. Image is upside down
        mask_yellow = self.threshold_yellow(image)  # hsv filter -> mask
        msg = Image()
        msg.data = bytes(mask_yellow.astype(np.uint8))
        msg.height = mask_yellow.shape[0]
        msg.width = mask_yellow.shape[1]
        msg.encoding = "mono8"
        msg.step = len(msg.data) // msg.height
        self.pub.publish(msg)

    def threshold_yellow(self, image):
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(image_hsv, YELLOW["lower"], YELLOW["upper"])
        return mask_yellow


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)
    YellowMask()
    rospy.spin()
