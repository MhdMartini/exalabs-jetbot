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
from jetbot import Camera
from sensor_msgs.msg import Image
import numpy as np


NODE_NAME = "camera_pub_node"
OUT_TOPIC = "camera"

# Jetbot original camera image dimensions
WIDTH = 3264
HEIGHT = 2464

DEFAULT_SCALE = 6

WIDTH_SCALED = int(WIDTH / DEFAULT_SCALE)  # 544
HEIGHT_SCALED = int(HEIGHT / DEFAULT_SCALE)  # 410


class CameraPub:
    def __init__(self):
        # create a jetbot.Camera instance. Images are stored in the "value" attribute of the instance
        pub = rospy.Publisher(OUT_TOPIC, Image, queue_size=1)
        self.camera = Camera.instance(width=WIDTH_SCALED, height=HEIGHT_SCALED)

        try:
            while True:
                image = self.camera.value
                image_ros = bytes(image.astype(np.uint8))  # cv_bridge alternative

                msg = Image()
                msg.data = image_ros
                msg.width = WIDTH_SCALED
                msg.height = HEIGHT_SCALED
                pub.publish(msg)
        except Exception as e:
            raise e
        finally:
            self.camera.stop()
            print("released camera")


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    CameraPub()
    rospy.spin()
