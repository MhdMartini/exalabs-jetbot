#!/usr/bin/env python3
"""
Node to subscribe to aruco_tags topic and publish the centroid for a specific id

Example use:
    Subscribes to: aruco_tags
        msg info:
            jetbot_msgs/ArucoTags

    Published to: aruco_centroid
        msg info:
            jetbot_msgs/Vector2D



Mohamed Martini
University of Massachusetts Lowell
"""
import numpy as np
import rospy
from jetbot_msgs.msg import Vector2D, ArucoTags
import os


class ArucoCentroid:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, ArucoTags, self.main, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, Vector2D, queue_size=1)

    def get_coords(self, tag):
        X = [tag.top_left.x, tag.top_right.x, tag.bottom_left.x, tag.bottom_right.x]
        Y = [tag.top_left.y, tag.top_right.y, tag.bottom_left.y, tag.bottom_right.y]
        return X, Y

    def get_centroid(self, x: list, y: list):
        x_centroid = np.mean(x)
        y_centroid = np.mean(y)
        return x_centroid, y_centroid

    def publish(self, centroid):
        msg = Vector2D()
        msg.x, msg.y = centroid
        self.pub.publish(msg)

    def main(self, msg):
        """
        get ArucoTags object, and publish the centroid of a wanted tag
        """
        tags = msg.tags
        for tag in tags:
            if tag.id != rospy.get_param(PARAM_ID, PARAM_ID_DEF):
                continue
            X, Y = self.get_coords(tag)
            centroid = self.get_centroid(X, Y)
            self.publish(centroid)


if __name__ == '__main__':
    NODE_NAME = "aruco_centroid_node"
    rospy.init_node(NODE_NAME)

    IN_TOPIC = "in_topic"
    OUT_TOPIC = "out_topic"

    PARAM_ID = os.path.join(rospy.get_name(), "TAG_ID")
    PARAM_ID_DEF = 0

    ArucoCentroid()
    rospy.spin()
