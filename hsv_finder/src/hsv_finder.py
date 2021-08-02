#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image

NODE_NAME = "hsv_finder_node"
IN_TOPIC = "camera/processed/cropped"
OUT_TOPIC_1 = "hsv_finder/rect"
OUT_TOPIC_2 = "hsv_finder/mask"

OFFSET = 16  # rect width 16*2
COLOR = (255, 0, 0)
THIKNESS = 2
YELLOW_HSV_PARAM = "HSV_YELLOW"


class HsvFinder:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.callback, queue_size=1)
        self.pub_1 = rospy.Publisher(OUT_TOPIC_1, Image, queue_size=1)
        self.pub_2 = rospy.Publisher(OUT_TOPIC_2, Image, queue_size=1)

    def callback(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)  # cv_bridge alternative
        # draw rectangle and publish the image
        # find min, max hsv values in rectangle and logward them
        start_point, end_point = self.draw_rect(img)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        self.find_hsv(hsv, start_point, end_point)

    def draw_rect(self, img):
        start_point = (img.shape[1] // 2 - OFFSET, 0)  # col, row
        end_point = (img.shape[1] // 2 + OFFSET, img.shape[0] - 1)
        rect = cv2.rectangle(img, start_point, end_point, COLOR, THIKNESS)
        self.publish(self.pub_1, rect)
        return start_point, end_point

    def find_hsv(self, hsv, start_point, end_point):
        start_col, start_row = start_point
        end_col, end_row = end_point
        hsv_cropped = hsv[start_row: end_row, start_col: end_col]
        h, s, v = cv2.split(hsv_cropped)
        lower = list(map(np.min, (h, s, v)))
        upper = list(map(np.max, (h, s, v)))
        hsv_vals = {"LOWER": np.array(lower), "UPPER": np.array(upper)}
        rospy.logwarn(hsv_vals)
        rospy.set_param(YELLOW_HSV_PARAM, hsv_vals)

        mask = cv2.inRange(hsv, np.array(hsv_vals["lower"]), np.array(hsv_vals["upper"]))
        self.publish(self.pub_2, mask, encoding="mono8")

    def publish(self, publisher, img, encoding="bgr8"):
        msg = Image()
        msg.data = bytes(img.astype(np.uint8))
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = encoding
        msg.step = len(msg.data) // msg.height
        publisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node(NODE_NAME)
    HsvFinder()
    rospy.spin()
