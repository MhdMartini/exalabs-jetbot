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


NODE_NAME = "motor_speed_node"
IN_TOPIC = "motor_speed"
OUT_TOPIC = "wheels_commands"


class _MotorSpeed:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, MotorSpeed, self.speed_to_wheels, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, WheelsCommands, queue_size=10)

    def speed_to_wheels(self, msg):
        # linear velocity v between -1, 1
        # angular velocity omega between -1, 1
        v, omega = msg.v, msg.omega
        left, right = -omega + v, omega + v

        WheelsCommands_msg = WheelsCommands()
        WheelsCommands_msg.left = left
        WheelsCommands_msg.right = right
        self.pub.publish(WheelsCommands_msg)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    _MotorSpeed()    
    rospy.spin()
