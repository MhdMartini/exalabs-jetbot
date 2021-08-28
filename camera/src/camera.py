#!/usr/bin/env python

# MIT License
# Copyright (c) 2019 JetsonHacks
# See license
# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit using OpenCV
# Drivers for the camera and OpenCV are included in the base image
import os
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 1280x720 @ 60fps
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of the window on the screen


def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


class Camera:
    def __init__(self):
        self.bridge = CvBridge()
        self.rate = rospy.Rate(1)
        self.cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        rospy.on_shutdown(lambda: self.cap.release())

        self.pub_raw = rospy.Publisher(OUT_TOPIC_1, Image, queue_size=1)
        self.pub_preprocessed = rospy.Publisher(OUT_TOPIC_2, Image, queue_size=1)
        self.pub_cropped = rospy.Publisher(OUT_TOPIC_3, Image, queue_size=1)

        self.stream()

    def resize(self, img, width, height):
        scale = rospy.get_param(PARAM_SCALE, PARAM_SCALE_DEF)
        return cv2.resize(img, (width // scale, height // scale), cv2.INTER_NEAREST)  # resize image

    def process(self, img):
        im_resize = self.resize(img=img, width=img.shape[1], height=img.shape[0])
        im_smooth = cv2.GaussianBlur(im_resize, (5, 5), 0)
        return im_smooth

    def crop(self, img):
        offset = rospy.get_param(PARAM_OFFSET, PARAM_OFFSET_DEF)
        return img[img.shape[0] // 2 - offset:, :]

    def stream(self):
        while not rospy.is_shutdown():
            if not self.cap.isOpened():
                rospy.logwarn("Unable to open camera")
                self.rate.sleep()
            ret_val, img = self.cap.read()
            self.pub_raw.publish(self.bridge.cv2_to_imgmsg(img, encoding="bgr8"))

            preprocessed = self.process(img)
            self.pub_preprocessed.publish(self.bridge.cv2_to_imgmsg(preprocessed, encoding="bgr8"))

            cropped = self.crop(preprocessed)
            self.pub_cropped.publish(self.bridge.cv2_to_imgmsg(cropped, encoding="bgr8"))


if __name__ == "__main__":
    NODE_NAME = "camera_node"
    rospy.init_node(NODE_NAME)

    OUT_TOPIC_1 = "out_topic_raw"
    OUT_TOPIC_2 = "out_topic_preprocessed"
    OUT_TOPIC_3 = "out_topic_cropped"

    PARAM_SCALE = os.path.join(rospy.get_name(), "SCALE")
    PARAM_SCALE_DEF = 4

    PARAM_OFFSET = os.path.join(rospy.get_name(), "OFFSET")
    PARAM_OFFSET_DEF = 40

    Camera()
