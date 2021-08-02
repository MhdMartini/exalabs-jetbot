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
PID_PARAM = "PID_ANGLE"
VIL_PARAM = "VELOCITY"

P = 0.19
I = 0
D = 0.4

V = 0  # vehicle velocity - 0.3


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
        pid = rospy.get_param(PID_PARAM)
        p, i, d = pid["P"], pid["I"], pid["D"]
        ctrl = p * p_err + i * i_err + d * d_err

        msg = MotorSpeed()
        msg.v = rospy.get_param(VIL_PARAM)
        msg.omega = ctrl
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.set_param(PID_PARAM, {"P": P, "I": I, "D": D})
    rospy.set_param(VIL_PARAM, V)
    PIDAngle()
    rospy.spin()
