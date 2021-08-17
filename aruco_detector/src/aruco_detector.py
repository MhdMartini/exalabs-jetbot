#! usr/env/bin python3
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
        rospy.Subscriber(IN_TOPIC, Image, self.main, queu_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, ArucoTags, queu_size=1)

    def aruco_detect(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        return corners, ids

    def publish(self, corners, ids):
        tags = ArucoTags()
        for corner, _id in zip(corners, ids):
            tag = ArucoTag()
            tag.id = _id
            corner = corner[0]
            top_left, top_right, bottom_right, bottom_left = corner

            tl = Vector2D()
            tl.x, tl.y = top_left
            tag.top_left = tl

            tr = Vector2D()
            tr.x, tr.y = top_right
            tag.top_right = tr

            br = Vector2D()
            br.x, br.y = bottom_right
            tag.bottom_right = br

            bl = Vector2D()
            bl.x, bl.y = bottom_left
            tag.bottom_left = bl

            tags.append(tag)

        self.pub.publish(tags)

    def main(self, msg):
        """
        get input image, get ids and corners of detected aruco tags and publish them
        """
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)  # cv_bridge alternative
        corners, ids = self.aruco_detect(image)
        self.publish(corners, ids)


if __name__ == '__main__':
    NODE_NAME = "aruco_detector_node"
    rospy.init_node(NODE_NAME)

    IN_TOPIC = "in_topic"
    OUT_TOPIC = "out_topic"

    _ArucoDetector()
    rospy.spin()
