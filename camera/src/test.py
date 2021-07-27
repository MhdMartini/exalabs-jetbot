#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

if __name__ == "__main__":
    rospy.init_node("test_node")
    bridge = CvBridge()
    pub = rospy.Publisher("test", Image, queue_size=1)
    #rospy.Subscriber("/jetbot/camera/raw", Image, lambda x: cv2.imwrite("/home/jetbot/Desktop/test.png", bridge.imgmsg_to_cv2(x.data, desired_encoding="bgr8")) )
    img = cv2.imread('/home/jetbot/Desktop/0.png')
    ros_img = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    pub.publish(ros_img)
    rospy.spin()
