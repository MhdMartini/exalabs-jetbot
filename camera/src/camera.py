#!/usr/bin/env python

# MIT License
# Copyright (c) 2019 JetsonHacks
# See license
# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit using OpenCV
# Drivers for the camera and OpenCV are included in the base image

import cv2
import rospy
from sensor_msgs.msg import Image
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


if __name__ == "__main__":
    NODE_NAME = "camera_node"
    rospy.init_node(NODE_NAME)

    OUT_TOPIC = "out_topic"
    pub = rospy.Publisher(OUT_TOPIC, Image, queue_size=1)

    rate = rospy.Rate(1)
    rospy.logwarn(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    rospy.on_shutdown(lambda x: cap.release())

    while not rospy.is_shutdown():
        if not cap.isOpened():
            rospy.logwarn("Unable to open camera")
            rate.sleep()

        ret_val, img = cap.read()
        pub.publish(img)
