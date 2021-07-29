#!/usr/bin/env python3
"""
Node to subscribe to angle error and publish control commands to the motor accordingly

Subscribes to: yellow_lane_angle
msg info:
    std_msgs/Float32

Published to: motor_speed
msg info:
    MotorSpeed
        v       Float32
        omega   Float32


Mohamed Martini
University of Massachusetts Lowell
"""

import rospy
from std_msgs.msg import Float32
from jetbot_msgs.msg import MotorSpeed
from pid import PID
import sys


NODE_NAME = "pid_angle_node"
IN_TOPIC = "yellow_lane_angle"
OUT_TOPIC = "motor_speed"

P = 0.3
I = 0
D = 0

V = 0.4  # vehicle velocity


class PIDAngle:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Float32, self.control, queue_size=1)

        self.pub = rospy.Publisher(OUT_TOPIC, MotorSpeed, queue_size=1)
        self.pid = None

    def control(self, msg):
        # receive angle error and control
        if self.pid is None:
            self.pid = PID()
        p_err, i_err, d_err = self.pid.add_error(msg.data)
        p, i, d = rospy.get_param('pid_angle')
        ctrl = p * p_err + i * i_err + d * d_err

        msg = MotorSpeed()
        msg.v = rospy.get_param("velocity")
        msg.omega = ctrl
        self.pub.publish(msg)

    def get_pid(self):
        return rospy.get_param('pid_angle')


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.set_param("pid_angle", [P, I, D])
    rospy.set_param("velocity", V)
    PIDAngle()
    rospy.spin()
