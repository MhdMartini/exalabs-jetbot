#!/usr/bin/env python3
"""
Node to receive an object's normalized centroid (0 to 1, 0 to 1) and publish its normalized angle (1 to -1).
If centroid is not found (blank mask) the publish centroid is -1, -1 and the publish angle is 0

Example use:
    Subscribes to: yellow_centroid
        msg info:
            jetbot_msgs/Vector2D

    Published to: yellow_angle
        msg info:
            std_msgs/Float32


Mohamed Martini
University of Massachusetts Lowell
"""

import rospy
from jetbot_msgs.msg import Vector2D
from std_msgs.msg import Float32


class ObjectAngle:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Vector2D, self.main, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, Float32, queue_size=1)

    def calc_angle(self, center_x):
        return (0.5 - center_x) / 0.5

    def main(self, msg):
        center_x = msg.x
        angle = self.calc_angle(center_x) if center_x != -1 else 0
        self.publish(angle)


if __name__ == "__main__":
    NODE_NAME = "yellow_lane_angle_node"
    rospy.init_node(NODE_NAME, anonymous=True)

    IN_TOPIC = "in_topic"
    OUT_TOPIC = "out_topic"
    ObjectAngle()
    rospy.spin()
