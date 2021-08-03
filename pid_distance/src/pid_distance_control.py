#!/usr/bin/env python3
"""
Node to subscribe to pid_distance errors and publish the control signals v.

Subscribes to: pid_distance
msg info:
    PIDError
        error   Float32
        i_error Float32
        d_error Float32

Publishes to: pid_distance_control
msg info:
    std_msgs/Float32
        v   Float32


Mohamed Martini
University of Massachusetts Lowell
"""

import rospy
from jetbot_msgs.msg import PIDError
from std_msgs.msg import Float32


NODE_NAME = "pid_distance_control_node"
IN_TOPIC = "pid_distance"
OUT_TOPIC = "pid_distance_control"

PARAM_PID = "PID_DISTANCE"

P, I, D = 0, 0, 0


class PIDDistanceControl:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, PIDError, self.calc_control, queue_size=10)
        self.pub = rospy.Publisher(OUT_TOPIC, Float32, queue_size=10)

    def calc_control(self, msg):
        pid_distance = msg.error, msg.i_error, msg.d_error  # angle errors
        gains = self.get_gains()  # errors gains
        v = self.get_v(pid_distance, gains)
        self.pub.publish(v)

    def get_gains(self):
        pid_gains = rospy.get_param(PARAM_PID)
        p, i, d = pid_gains["P"], pid_gains["I"], pid_gains["D"]
        return p, i, d

    def get_v(self, pid_distance, gains):
        err, i_err, d_err = pid_distance
        p, i, d = gains
        return p * err + i * i_err + d * d_err


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    rospy.set_param(PARAM_PID, {"P": P, "I": I, "D": D})
    PIDDistanceControl()
    rospy.spin()
