import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image


calibration = cv2.imread("calibration.png", 0)
NODE_NAME = "hsv_finder_node"
IN_TOPIC = "camera/processed/cropped"
FOUND = False


def find_hsv(msg):
    h_min = 0
    h_max = 180
    s_min = 0
    s_max = 256
    v_min = 0
    v_max = 256
    hsv_vals = {
        "h_min": None,
        "h_max": None,
        "s_min": None,
        "s_max": None,
        "v_min": None,
        "v_max": None,
    }

    in_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)  # cv_bridge alternative
    hsv = cv2.cvtColor(in_img, cv2.COLOR_BGR2HSV)

    val = 0
    prev = 0
    for h_min in range(h_min, h_max):
        lower = np.array([h_min, s_min, v_min])
        higher = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, higher)
        val_mask = np.sum(cv2.bitwise_and(calibration, mask))
        if val_mask < val:
            hsv_vals["h_min"] = prev
            break
        val = val_mask
        prev = h_min

    val = 0
    prev = 0
    for h_max in range(h_max, h_min, -1):
        lower = np.array([h_min, s_min, v_min])
        higher = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, higher)
        val_mask = np.sum(cv2.bitwise_and(calibration, mask))
        if val_mask < val:
            hsv_vals["h_max"] = prev
            break
        val = val_mask
        prev = h_max

    val = 0
    prev = 0
    for s_min in range(s_min, s_max):
        lower = np.array([h_min, s_min, v_min])
        higher = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, higher)
        val_mask = np.sum(cv2.bitwise_and(calibration, mask))
        if val_mask < val:
            hsv_vals["s_min"] = prev
            break
        val = val_mask
        prev = s_min

    val = 0
    prev = 0
    for s_max in range(s_max, s_min, -1):
        lower = np.array([h_min, s_min, v_min])
        higher = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, higher)
        val_mask = np.sum(cv2.bitwise_and(calibration, mask))
        if val_mask < val:
            hsv_vals["s_max"] = prev
            break
        val = val_mask
        prev = s_max

    val = 0
    prev = 0
    for v_min in range(v_min, v_max):
        lower = np.array([h_min, s_min, v_min])
        higher = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, higher)
        val_mask = np.sum(cv2.bitwise_and(calibration, mask))
        if val_mask < val:
            hsv_vals["v_min"] = prev
            break
        val = val_mask
        prev = v_min

    val = 0
    prev = 0
    for v_max in range(v_max, v_min, -1):
        lower = np.array([h_min, s_min, v_min])
        higher = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, higher)
        val_mask = np.sum(cv2.bitwise_and(calibration, mask))
        if val_mask < val:
            hsv_vals["v_max"] = prev
            break
        val = val_mask
        prev = v_max

    rospy.logwarn(f"HSV values: {hsv_vals}")
    FOUND = True
    return


if __name__ == "__main__":

    rospy.init_node(NODE_NAME)
    rospy.Subscriber(IN_TOPIC, Image, find_hsv, queue_size=1)
    while not FOUND:
        rospy.rostime.wallsleep(0.5)
