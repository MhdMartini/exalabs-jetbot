#!/usr/bin/env python3
"""
Node to publish jetbot processed camera images.

Subscribes to: camera/raw
msg info:
    sensor_msgs/Image

Published to: camera/processed
msg info:
    sensor_msgs/Image

Mohamed Martini
University of Massachusetts Lowell
"""
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np


NODE_NAME = "img_processed_node"
IN_TOPIC = "camera/raw"
OUT_TOPIC = "camera/processed"

SCALE = 2

class ImgProcess:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.process, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, Image, queue_size=1)

    def process(self, msg):
        im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)  # cv_bridge alternative
        im = cv2.rotate(im, cv2.ROTATE_180)  # rotate image 180
        im = cv2.resize(im, (msg.width // SCALE, msg.height // SCALE))  # resize image
        im_smooth = cv2.GaussianBlur(im, (5, 5), 0)
        self.publish(im_smooth)

    def publish(self, img):
        msg = Image()
        msg.data = bytes(img.astype(np.uint8))
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = "rgb8"
        msg.step = len(msg.data) // msg.height
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    ImgProcess()
    rospy.spin()
