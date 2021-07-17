#!/usr/bin/env python3
"""
Node to publish jetbot camera images.

Subscribes to: None
Published to: camera
msg info:
    sensor_msgs/Image
        data    ros image msg

Mohamed Martini
University of Massachusetts Lowell
"""
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np


NODE_NAME = "img_test_node"
#IN_TOPIC = "/jetbot/camera/raw"  # add /jetbot/ namespace when only running with rosrun
IN_TOPIC = "/jetbot/camera/yellow_mask"


class ImgTest:
    def __init__(self):
        # create a jetbot.Camera instance. Images are stored in the "value" attribute of the instance

        rospy.Subscriber(IN_TOPIC, Image, self.callback, queue_size=1)
        self.i = 0

    def callback(self, msg):
        im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)  # cv_bridge alternative
        cv2.imwrite("/home/jetbot/Desktop/test{}.png".format(str(self.i % 5)), im)
        self.i += 1
        rospy.logwarn("Saving Images to your Desktop")

if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    ImgTest()
    rospy.spin()
