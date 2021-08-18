#!/usr/bin/env python3
"""
Node to detect aruco tags and publish

Example use:
    Subscribes to: camera/processed
        msg info:
            sensor_msgs/Image

    Published to: aruco_detector
        msg info:
            jetbot_msgs/ArucoTags :-> list of ArucoTag objects



Mohamed Martini
University of Massachusetts Lowell
"""
import numpy as np
import cv2
from cv2 import aruco
import rospy
from sensor_msgs.msg import Image
from jetbot_msgs.msg import Vector2D, ArucoTag, ArucoTags


class _ArucoDetector:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.main, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, ArucoTags, queue_size=1)

    def aruco_detect(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        return corners, ids

    def get_tags(self, corners, ids, height, width):
        tags = []
        for corner, _id in zip(corners, ids):
            tag = ArucoTag()
            tag.id = _id[0]
            corner = corner[0]
            top_left, top_right, bottom_right, bottom_left = corner

            tl = Vector2D()
            tl.x, tl.y = top_left
            tl.x /= width
            tl.y /= height
            tag.top_left = tl

            tr = Vector2D()
            tr.x, tr.y = top_right
            tr.x /= width
            tr.y /= height
            tag.top_right = tr

            br = Vector2D()
            br.x, br.y = bottom_right
            br.x /= width
            br.y /= height
            tag.bottom_right = br

            bl = Vector2D()
            bl.x, bl.y = bottom_left
            bl.x /= width
            bl.y /= height
            tag.bottom_left = bl

            tags.append(tag)
        return tags

    def tags_to_msg(self, tags):
        tags_msg = ArucoTags()
        tags_msg.tags = tags
        return tags_msg

    def publish(self, tags_msg):
        self.pub.publish(tags_msg)

    def main(self, msg):
        """
        get input image, get ids and corners of detected aruco tags and publish them
        """
        height, width = msg.height, msg.width
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, -1)  # cv_bridge alternative
        corners, ids = self.aruco_detect(image)
        if ids is None:
            return
        tags = self.get_tags(corners, ids, height, width)
        tags_msg = self.tags_to_msg(tags)
        self.publish(tags_msg)


if __name__ == '__main__':
    NODE_NAME = "aruco_detector_node"
    rospy.init_node(NODE_NAME)

    IN_TOPIC = "in_topic"
    OUT_TOPIC = "out_topic"

    _ArucoDetector()
    rospy.spin()
