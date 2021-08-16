#!/usr/bin/env python3
"""
Lane following Node. Subscribes to error control topics and publishes to the motor

Example use:
    Subscribes to: pid_angle_control
        msg info:
            std_msgs/Float32
                omega   Float32

    Publishes to: motor_speed
        msg info:
            jetbot_msgs/MotorSpeed
                v       Float32
                omega   Float32


Mohamed Martini
University of Massachusetts Lowell
"""

import rospy
from jetbot_msgs.msg import MotorSpeed
from std_msgs.msg import Float32
import os


class LaneFollowingAngleOnly:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Float32, self.main, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, MotorSpeed, queue_size=1)

    def get_v(self, omega):
        SHARP_TURN = rospy.get_param(PARAM_SHARP_TURN, PARAM_SHARP_TURN_DEF)
        if not (-SHARP_TURN <= omega <= SHARP_TURN):
            return rospy.get_param(PARAM_MIN_VEL, PARAM_MIN_VEL_DEF)
        return rospy.get_param(PARAM_MAX_VEL, PARAM_MAX_VEL_DEF)

    def publish(self, v, omega):
        msg = MotorSpeed()
        msg.v, msg.omega = v, omega
        self.pub.publish(msg)

    def main(self, msg):
        # receive the control omega; calculate v accirdingly; publish control signals
        omega = msg.data
        v = self.get_v(omega)
        self.publish(v, omega)


if __name__ == '__main__':
    NODE_NAME = "lane_following_angle_only_node"
    rospy.init_node(NODE_NAME)

    IN_TOPIC = "pid_control_slope"
    OUT_TOPIC = "motor_speed"

    PARAM_MAX_VEL = os.path.join(rospy.get_name(), "MAX_VELOCITY")
    PARAM_MAX_VEL_DEF = 0.32

    PARAM_MIN_VEL = os.path.join(rospy.get_name(), "MIN_VELOCITY")
    PARAM_MIN_VEL_DEF = 0.25

    PARAM_SHARP_TURN = os.path.join(rospy.get_name(), "SHARP_TURN")
    PARAM_SHARP_TURN_DEF = 0.1

    PARAM_CTRL_READY = os.path.join(rospy.get_name(), "CONTROLLER_READY")
    PARAM_CTRL_READY_DEF = 0

    rospy.set_param(PARAM_CTRL_READY, 0)
    rate = rospy.Rate(1)
    while rospy.get_param(PARAM_CTRL_READY) != 1:
        rospy.logwarn(f"Waiting for 'CONTROLLER_READY' to be 1")
        rate.sleep()

    if rospy.get_param(PARAM_CTRL_READY) == 1:
        # this is in case Ctrl+C is used while in the while loop
        LaneFollowingAngleOnly()
        rospy.spin()