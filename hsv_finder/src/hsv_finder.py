#!/usr/bin/env python3
"""
Node to subscribe to the camera/processed/cropped image and publish the detected HSV values

Subscribes to: camera/processed/cropped
msg info:
    sensor_msgs/Image

Published to: None

Sets the required HSV parameter (HSV_YELLOW, HSV_BLUE, HSV_WHITE, etc.), or the parameter HSV_FINDER if not specified

Examples:
    rosrun hsv_finder hsv_finder.py yellow
        sets the HSV_YELLOW parameter
    rosrun hsv_finder hsv_finder.py blue
        sets the HSV_BLUE parameter
    rosrun hsv_finder hsv_finder.py
        sets the HSV_FINDER parameter

Mohamed Martini
University of Massachusetts Lowell
"""
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
import sys
import os

NAME_SPACE = "/jetbot"


class HSVFinder:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.main, queue_size=1)
        self.pub_1 = rospy.Publisher(OUT_TOPIC_1, Image, queue_size=1)
        self.pub_2 = rospy.Publisher(OUT_TOPIC_2, Image, queue_size=1)

    def change_set_parameter(self):
        pass

    def get_rect_points(self, height, width):
        offset = rospy.get_param(PARAM_OFFSET)
        rect_width = rospy.get_param(PARAM_RECT_WIDTH)

        # left rectangle
        start_left = (0, offset)  # col, row
        end_left = (rect_width // 2, height - 1)

        # mid rectangle
        start_center = (width // 2 - rect_width // 2, offset)
        end_center = (width // 2 + rect_width // 2, height - 1)

        # right rectangle
        start_right = (width - rect_width // 2, offset)
        end_right = (width - 1, height - 1)

        return (start_left, start_center, start_right), (end_left, end_center, end_right)

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
        rospy.set_param(rospy.get_param(PARAM_PARAM_TO_SET), hsv_vals)

    def main(self, msg):
        height, width = msg.height, msg.width
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, -1)  # cv_bridge alternative
        img_rect = np.copy(img)

        params_rect = (PARAM_RECT_LEFT, PARAM_RECT_CENTER, PARAM_RECT_RIGHT)
        start_points, end_points = self.get_rect_points(height, width)
        for param, start_point, end_point in zip(params_rect, start_points, end_points):
            if not rospy.get_param(param):
                continue
            img_rect = cv2.rectangle(img_rect, start_point, end_point, COLOR, THIKNESS)
        self.publish(self.pub_1, img_rect)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_vals = self.find_hsv(hsv, start_points, end_points)
        self.update_hsv(hsv_vals)

        mask = cv2.inRange(hsv, np.array(hsv_vals["LOWER"]), np.array(hsv_vals["UPPER"]))
        self.publish(self.pub_2, mask, encoding="mono8")


if __name__ == "__main__":
    NODE_NAME = "hsv_finder_node"
    rospy.init_node(NODE_NAME)

    IN_TOPIC = os.path.join(NAME_SPACE, "camera/processed/cropped")

    OUT_TOPIC_1 = os.path.join(NAME_SPACE, rospy.get_name(), "rect")
    OUT_TOPIC_2 = os.path.join(NAME_SPACE, rospy.get_name(), "mask")

    PARAM_PARAM_TO_SET = os.path.join(NAME_SPACE, rospy.get_name(), "PARAM_TO_SET")
    PARAM_PARAM_TO_SET_DEF = "none"
    rospy.set_param(PARAM_PARAM_TO_SET, PARAM_PARAM_TO_SET_DEF)

    PARAM_RECT_WIDTH = os.path.join(NAME_SPACE, rospy.get_name(), "RECT_WIDTH")
    PARAM_RECT_WIDTH_DEF = 26  # width of rectangle
    rospy.set_param(PARAM_RECT_WIDTH, PARAM_RECT_WIDTH_DEF)

    PARAM_OFFSET = os.path.join(NAME_SPACE, rospy.get_name(), "OFFSET")
    PARAM_OFFSET_DEF = 40  # all height but top 40 pixles
    rospy.set_param(PARAM_OFFSET, PARAM_OFFSET_DEF)

    PARAM_RECT_LEFT = os.path.join(NAME_SPACE, rospy.get_name(), "RECT_LEFT")
    PARAM_RECT_LEFT_DEF = 0
    rospy.set_param(PARAM_RECT_LEFT, PARAM_RECT_LEFT_DEF)

    PARAM_RECT_CENTER = os.path.join(NAME_SPACE, rospy.get_name(), "RECT_CENTER")
    PARAM_RECT_CENTER_DEF = 1
    rospy.set_param(PARAM_RECT_CENTER, PARAM_RECT_CENTER_DEF)

    PARAM_RECT_RIGHT = os.path.join(NAME_SPACE, rospy.get_name(), "RECT_RIGHT")
    PARAM_RECT_RIGHT_DEF = 0
    rospy.set_param(PARAM_RECT_RIGHT, PARAM_RECT_RIGHT_DEF)

    COLOR = (255, 0, 0)  # rect color and thikness
    THIKNESS = 2

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        if rospy.get_param(PARAM_PARAM_TO_SET) != "none":
            break
        rospy.logwarn("Please set the 'PARAM_TO_SET' parameter with the full name of the HSV parameter you wish to modify")
        rate.sleep()

    HSVFinder()
    rospy.spin()
