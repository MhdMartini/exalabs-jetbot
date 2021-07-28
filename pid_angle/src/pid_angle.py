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


NODE_NAME = "pid_angle_node"
IN_TOPIC = "yellow_lane_angle"
OUT_TOPIC = "motor_speed"

P = 1
I = 0
D = 0

V = 0.5  # vehicle velocity


class PIDAngle:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Float32, self.control, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, MotorSpeed, queue_size=1)
        self.pid = None

    def control(self, msg):
        # receive angle error and control
        if self.pid is None:
            self.pid = PID(msg.data)
        p_err, i_err, d_err = self.pid.add_error(msg.data)
        ctrl = P * p_err + I * i_err + D * d_err

        msg = MotorSpeed()
        msg.v = V
        msg.omega = - ctrl
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)
    PIDAngle()
    rospy.spin()
