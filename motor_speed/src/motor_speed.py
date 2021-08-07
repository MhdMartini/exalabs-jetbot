#!/usr/bin/env python3
"""
Node to control the Jetbot's motor linear and angular speed.

Subscribes to: motor_speed
msg info:
    MotorSpeed
        v       Float32
        omega   Float32

Published to: wheels_commands
msg info:
    WheelsCommands
        left    Float32
        right   Float32


Mohamed Martini
University of Massachusetts Lowell
"""
import rospy
from jetbot_msgs.msg import MotorSpeed, WheelsCommands
import os


class _MotorSpeed:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, MotorSpeed, self.speed_to_wheels, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, WheelsCommands, queue_size=10)
        rospy.on_shutdown(self.stop)

    def speed_to_wheels(self, msg):
        # linear velocity v between -1, 1
        # angular velocity omega between -1, 1
        v, omega = msg.v, msg.omega + rospy.get_param(TRIM_PARAM, TRIM_PARAM_DEF)
        self.command(v, omega)

    def command(self, v, omega):
        left, right = -omega + v, omega + v
        WheelsCommands_msg = WheelsCommands()
        WheelsCommands_msg.left = left
        WheelsCommands_msg.right = right
        self.pub.publish(WheelsCommands_msg)

    def stop(self):
        self.command(0, 0)
        rospy.logwarn("MOTOR STOPPED")


if __name__ == '__main__':
    NODE_NAME = "motor_speed_node"
    rospy.init_node(NODE_NAME, anonymous=True)

    IN_TOPIC = "motor_speed"
    OUT_TOPIC = "wheels_commands"

    TRIM_PARAM = os.path.join(os.get_name(), "TRIM")
    TRIM_PARAM_DEF = -0.014

    _MotorSpeed()
    rospy.spin()
