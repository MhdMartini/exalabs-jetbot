#!/usr/bin/env python3
"""
Lane following Node. Subscribes to error control topics and publishes to the motor.
Jetbot will be able to follow the yellow lane and stop at red marks

Mohamed Martini
University of Massachusetts Lowell
"""

import rospy
from jetbot_msgs.msg import MotorSpeed
from std_msgs.msg import Float32
import os


class LaneFollowingDemo1:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC_SLOPE, Float32, self.main, queue_size=1)
        rospy.Subscriber(IN_TOPIC_DIST, Float32, self.distance, queue_size=1)

        self.pub = rospy.Publisher(OUT_TOPIC, MotorSpeed, queue_size=1)

        self.omega = 0
        self.slow_down = 0

    def distance(self, msg):
        if msg.data < 0:
            self.slow_down = msg.data
        else:
            self.slow_down = 0

    def get_v(self, omega):
        if abs(omega) <= rospy.get_param(PARAM_SHARP_TURN, PARAM_SHARP_TURN_DEF):
            v = rospy.get_param(PARAM_MAX_VEL, PARAM_MAX_VEL_DEF) + self.slow_down
        else:
            v = rospy.get_param(PARAM_MIN_VEL, PARAM_MIN_VEL_DEF) + self.slow_down
        return max(v, 0)

    def publish(self, v, omega):
        msg = MotorSpeed()
        msg.v = v
        msg.omega = omega
        self.pub.publish(msg)

    def main(self, msg):
        omega = msg.data
        v = self.get_v(omega)
        self.publish(v, omega)


if __name__ == '__main__':
    NODE_NAME = "lane_following_demo_1_node"
    rospy.init_node(NODE_NAME)

    IN_TOPIC_SLOPE = "pid_control_slope"
    IN_TOPIC_DIST = "pid_control_distance"

    OUT_TOPIC = "motor_speed"

    PARAM_MAX_VEL = os.apth.join(rospy.get_name(), "MAX_VELOCITY")
    PARAM_MAX_VEL_DEF = 0.26

    PARAM_MIN_VEL = os.apth.join(rospy.get_name(), "MIN_VELOCITY")
    PARAM_MIN_VEL_DEF = 0.26

    PARAM_SHARP_TURN = PARAM_MIN_VEL = os.apth.join(rospy.get_name(), "SHARP_TURN")
    PARAM_SHARP_TURN_DEF = 0.01

    PARAM_CTRL_READY = os.apth.join(rospy.get_name(), "CONTROLLER_READY")
    PARAM_CTRL_READY_DEF = 0

    rospy.set_param(PARAM_CTRL_READY, PARAM_CTRL_READY_DEF)
    rate = rospy.Rate(1)
    while rospy.get_param(PARAM_CTRL_READY) != 1:
        rospy.logwarn(f"Waiting for 'CONTROLLER_READY' to be 1")
        rate.sleep()

    if rospy.get_param(PARAM_CTRL_READY) == 1:
        # this is in case Ctrl+C is used while in the while loop
        LaneFollowingDemo1()
        rospy.spin()
