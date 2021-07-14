#!/usr/bin/env python3
"""
Node to publish jetbot camera images.

Subscribes to: None
Published to: camera
msg info:
    CameraImage
        data    list[]


Mohamed Martini
University of Massachusetts Lowell
"""
import rospy
from jetbot import Camera
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


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
        bridge = CvBridge()
        pub = rospy.Publisher(OUT_TOPIC, Image, queue_size=1)

        self.camera = Camera.instance(width=WIDTH_SCALED, height=HEIGHT_SCALED)

        try:
            while True:
                image = self.camera.value
                image_ros = bridge.cv2_to_imgmsg(image, "bgr8")
                pub.publish(image_ros)
        except Exception as e:
            self.camera.stop()
            raise e


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    CameraPub()
    rospy.spin()
