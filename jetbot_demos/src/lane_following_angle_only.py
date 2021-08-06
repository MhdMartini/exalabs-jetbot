#!/usr/bin/env python3
"""
Node to subscribe to pid_angle errors and publish the control signals omega.

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


NODE_NAME = "lane_following_angle_only_node"
IN_TOPIC = "pid_control_angle"
OUT_TOPIC = "motor_speed"

MAX_VELOCITY = 0.32
MIN_VELOCITY = 0.25
SHARP_TURN = 0.1

PARAM_MAX_VELOCITY = "MAX_VELOCITY"
PARAM_MIN_VELOCITY = "MIN_VELOCITY"
PARAM_CTRL_READY = "CONTROLLER_READY"


class LaneFollowingAngleOnly:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Float32, self.main, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, MotorSpeed, queue_size=1)

    def get_v(self, omega):
        if not (-SHARP_TURN <= omega <= SHARP_TURN):
            return rospy.get_param(PARAM_MIN_VELOCITY)
        return rospy.get_param(PARAM_MAX_VELOCITY)

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
    rospy.init_node(NODE_NAME)

    rospy.set_param(PARAM_CTRL_READY, 0)
    rate = rospy.Rate(1)
    while rospy.get_param(PARAM_CTRL_READY) != 1:
        rospy.logwarn(f"Waiting for {PARAM_CTRL_READY} to be 1")
        rate.sleep()

    if rospy.get_param(PARAM_CTRL_READY) == 1:
        # this is in case Ctrl+C is used while in the while loop
        rospy.set_param(PARAM_MIN_VELOCITY, MIN_VELOCITY)
        rospy.set_param(PARAM_MAX_VELOCITY, MAX_VELOCITY)
        LaneFollowingAngleOnly()
        rospy.spin()
