#!/usr/bin/env python3
"""
Node to control the individual wheels of the Jetbot.

Subscribes to: wheels_commands
msg info:
    WheelsCommands
        left    Float32
        right   Float32

Publishes to: None

Mohamed Martini
University of Massachusetts Lowell
"""
import rospy
from jetbot_msgs.msg import WheelsCommands
from jetbot import Robot

NODE_NAME = "wheels_commands_node"
IN_TOPIC = "wheels_commands"


class _WheelsCommands:
    def __init__(self, robot=None):
        # input option of passing an already created robot instance
        if robot is None:
            robot = Robot()
        self.robot = robot
        self.robot.set_motors(0, 0)

        rospy.Subscriber(IN_TOPIC, WheelsCommands, self.command_wheels, queue_size=1)

    def command_wheels(self, msg):
        # receive WheelsCommands msg and send commands to individual
        # ignore invalid commands
        left, right = msg.left, msg.right
        if not (-1 <= left <= 1):
            rospy.logwarn(f"Invalid speed value {left} for left motor. Speed values should fall between 0 and 1.")
            left = 0
        if not (-1 <= right <= 1):
            rospy.logwarn(f"Invalid speed value {right} for right motor. Speed values should fall between 0 and 1.")
            right = 0
        self.robot.set_motors(left, right)

        rospy.loginfo(f"left wheel speed: {left}")
        rospy.loginfo(f"right_wheel speed: {right}")


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    _WheelsCommands()
    rospy.spin()
