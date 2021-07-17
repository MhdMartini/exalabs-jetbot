#!/usr/bin/env python3
"""
Node to find the angle between the jetbot and the center of the detected yellow lane

Subscribes to: camera/yellow_mask
msg info:
    sensor_msgs/Image

Published to: yellow_lane_angle
msg info:
    jetbot_msgs/Vector2D


Mohamed Martini
University of Massachusetts Lowell
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


NODE_NAME = "yellow_lane_angle_node"
IN_TOPIC = "camera/yellow_mask"
OUT_TOPIC = "yellow_lane_angle"


class YellowLaneAngle:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Image, self.find_angle, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, Float32, queue_size=1)

    def find_angle(self, msg):
        # find the relative horizontal angle of the yellow lane [1 -> -1]
        yellow_mask = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)  # cv_bridge alternative
        canter_x_pixel = int(msg.width / 2)
        center_x = self.get_center_x(yellow_mask)
        if center_x is None:
            angle = 0
        else:
            angle = (center_x - canter_x_pixel) / canter_x_pixel  # (center_x - canter_x_pixel) should be (canter_x_pixel - center_x) if camera image is not flipped

        self.pub.publish(angle)

    def get_center_x(self, yellow_mask):
        moments = cv2.moments(yellow_mask)
        try:
            return int(moments["m10"] / moments["m00"])
        except ZeroDivisionError:  # no lane is detected
            return None


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)
    YellowLaneAngle()
    rospy.spin()
