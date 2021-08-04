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
YELLOW_HSV_PARAM = "HSV_YELLOW"

YELLOW = {
    # hsv range of yellow color.
    # IMPORTANT: Change for different lighting conditions using rosrun hsv_filter hsv_filter.py to set the hsv_yellow parameter
    # Make sure the camera node is running, and this node is also running (or whatever node that uses the hsv_yellow param) to make use of the new hsv values.
    # Otherwise, the values will just be logwarned, and the set parameter will not be used by any node
    "LOWER": (0, 98, 139),
    "UPPER": (179, 255, 255)
}


class YellowMask:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.yellow_filter, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, Image, queue_size=1)

    def yellow_filter(self, msg):
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)  # cv_bridge alternative. Image is upside down
        mask_yellow = self.threshold_yellow(image)  # hsv filter -> mask
        self.publish(mask_yellow)

    def threshold_yellow(self, image):
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_vals = rospy.get_param(YELLOW_HSV_PARAM)
        mask_yellow = cv2.inRange(image_hsv, np.array(hsv_vals["LOWER"]), np.array(hsv_vals["UPPER"]))
        return mask_yellow

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
    rospy.set_param(YELLOW_HSV_PARAM, YELLOW)
    YellowMask()
    rospy.spin()
