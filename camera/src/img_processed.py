#!/usr/bin/env python3
"""
Node to publish jetbot processed camera images.

Subscribes to: camera/raw
msg info:
    sensor_msgs/Image

Published to: camera/processed; camera/processed/cropped
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
OUT_TOPIC_1 = "camera/processed"
OUT_TOPIC_2 = "camera/processed/cropped"

SCALE = 2

class ImgProcess:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.process, queue_size=1)
        self.pub1 = rospy.Publisher(OUT_TOPIC_1, Image, queue_size=1)
        self.pub2 = rospy.Publisher(OUT_TOPIC_2, Image, queue_size=1)

    def process(self, msg):
        im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)  # cv_bridge alternative
        im = cv2.rotate(im, cv2.ROTATE_180)  # rotate image 180
        im = cv2.resize(im, (msg.width // SCALE, msg.height // SCALE))  # resize image
        im_smooth = cv2.GaussianBlur(im, (5, 5), 0)
        self.publish(self.pub1, im_smooth)
        cropped = im_smooth[im_smooth.shape[0]//2: , :]
        self.publish(self.pub2, cropped)

    def publish(self, publisher, img):
        msg = Image()
        msg.data = bytes(img.astype(np.uint8))
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = "bgr8"
        msg.step = len(msg.data) // msg.height
        publisher.publish(msg)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    ImgProcess()
    rospy.spin()
