#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image

NODE_NAME = "hsv_finder_node"
IN_TOPIC = "/jetbot/camera/processed/cropped"
OUT_TOPIC_1 = "/jetbot/hsv_finder/rect"
OUT_TOPIC_2 = "/jetbot/hsv_finder/mask"

OFFSET = 13  # rect width 13*2
COLOR = (255, 0, 0)
THIKNESS = 2
YELLOW_HSV_PARAM = "/jetbot/HSV_YELLOW"


class HSVFinder:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.main, queue_size=1)
        self.pub_1 = rospy.Publisher(OUT_TOPIC_1, Image, queue_size=1)
        self.pub_2 = rospy.Publisher(OUT_TOPIC_2, Image, queue_size=1)

    def get_rect_points(self, height, width):
        # left rectangle
        start_left = (0, 0)  # col, row
        end_left = (OFFSET, height - 1)

        # mid rectangle
        start_mid = (width // 2 - OFFSET, 0)
        end_mid = (width // 2 + OFFSET, height - 1)

        # right rectangle
        start_right = (width - OFFSET, 0)
        end_right = (width - 1, height - 1)

        return (start_left, start_mid, start_right), (end_left, end_mid, end_right)

    def draw_rects(self, img, start_points, end_points):
        for start_point, end_point in zip(start_points, end_points):
            img = cv2.rectangle(img, start_point, end_point, COLOR, THIKNESS)
        return img

    def publish(self, publisher, img, encoding="bgr8"):
        msg = Image()
        msg.data = bytes(img.astype(np.uint8))
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = encoding
        msg.step = len(msg.data) // msg.height
        publisher.publish(msg)

    def find_hsv(self, hsv, start_points, end_points):
        stack = []
        for start_point, end_point in zip(start_points, end_points):
            start_col, start_row = start_point
            end_col, end_row = end_point
            hsv_cropped = hsv[start_row: end_row, start_col: end_col]
            stack.append(hsv_cropped)

        np_stack = np.hstack(stack)
        h, s, v = cv2.split(np_stack)
        lower = [float(np.min(channel)) for channel in (h, s, v)]
        upper = [float(np.max(channel)) for channel in (h, s, v)]

        return {"LOWER": lower, "UPPER": upper}

    def update_hsv(self, hsv_vals):
        rospy.logwarn(hsv_vals)
        rospy.set_param(YELLOW_HSV_PARAM, hsv_vals)

    def main(self, msg):
        height, width = msg.height, msg.width
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, -1)  # cv_bridge alternative

        start_points, end_points = self.get_rect_points(height, width)  # find rectangles' coordinates
        rects = self.draw_rects(img, start_points, end_points)  # draw rectangles on image
        self.publish(self.pub_1, rects)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_vals = self.find_hsv(hsv, start_points, end_points)
        self.update_hsv(hsv_vals)

        mask = cv2.inRange(hsv, np.array(hsv_vals["LOWER"]), np.array(hsv_vals["UPPER"]))
        self.publish(self.pub_2, mask, encoding="mono8")


if __name__ == "__main__":
    rospy.init_node(NODE_NAME)
    HSVFinder()
    rospy.spin()
