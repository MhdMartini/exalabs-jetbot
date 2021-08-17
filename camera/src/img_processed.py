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
import os


class ImgProcess:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.main, queue_size=1)
        self.pub1 = rospy.Publisher(OUT_TOPIC_1, Image, queue_size=1)
        self.pub2 = rospy.Publisher(OUT_TOPIC_2, Image, queue_size=1)

    def resize(self, im, width, height):
        scale = rospy.get_param(PARAM_SCALE, PARAM_SCALE_DEF)
        return cv2.resize(im, (width // scale, height // scale), cv2.INTER_NEAREST)  # resize image

    def publish(self, img, publisher):
        msg = Image()
        msg.data = bytes(img.astype(np.uint8))
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = "bgr8"
        msg.step = len(msg.data) // msg.height
        publisher.publish(msg)

    def crop(self, im):
        offset = rospy.get_param(PARAM_OFFSET, PARAM_OFFSET_DEF)
        return im[im.shape[0] // 2 - offset:, :]

    def main(self, msg):
        im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)  # cv_bridge alternative
        im = self.resize(im, msg.width, msg.height)
        im = cv2.rotate(im, cv2.ROTATE_180)
        im_smooth = cv2.GaussianBlur(im, (5, 5), 0)
        self.publish(im_smooth, self.pub1)

        cropped = self.crop(im_smooth)
        self.publish(cropped, self.pub2)


if __name__ == '__main__':

    NODE_NAME = "img_processed_node"
    rospy.init_node(NODE_NAME, anonymous=True)

    IN_TOPIC = "camera/raw"
    OUT_TOPIC_1 = "camera/processed"
    OUT_TOPIC_2 = "camera/processed/cropped"

    PARAM_SCALE = os.path.join(rospy.get_name(), "SCALE")
    PARAM_SCALE_DEF = 2

    PARAM_OFFSET = os.path.join(rospy.get_name(), "OFFSET")
    PARAM_OFFSET_DEF = 40

    ImgProcess()
    rospy.spin()
