#!/usr/bin/env python3
"""
Node to filter out and publish a thresholded yellow lane image

Example use:
    Subscribes to: camera/processed/cropped
    msg info:
        sensor_msgs/Image

    Published to: yellow_mask
    msg info:
        sensor_msgs/Image


    Mohamed Martini
    University of Massachusetts Lowell
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image


class ColorMask:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.main, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, Image, queue_size=1)

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

    def filter_mask(self, mask):
        erode = cv2.erode(mask, KERNEL)
        return cv2.dilate(erode, KERNEL)

    def main(self, msg):
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)  # cv_bridge alternative
        mask = self.threshold_color(image)  # hsv filter -> mask

        if rospy.get_param(PARAM_NOISE_CANCEL, PARAM_NOISE_CANCEL_DEF) != 0:
            mask = self.filter_mask(mask)
        self.publish(mask)


if __name__ == "__main__":
    NODE_NAME = "color_mask_node"
    rospy.init_node(NODE_NAME, anonymous=True)

    IN_TOPIC = "in_topic"
    OUT_TOPIC = "out_topic"

    PARAM_HSV = "~/HSV"
    PARAM_HSV_DEF = {
        # yellow
        "LOWER": (1, 110, 150),
        "UPPER": (120, 255, 255)
    }

    PARAM_NOISE_CANCEL = "~/NOISE_CANCEL"
    PARAM_NOISE_CANCEL_DEF = 0

    KERNEL = np.ones((4, 4), np.uint8)

    ColorMask()
    rospy.spin()
