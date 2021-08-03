#!/usr/bin/env python3
"""
Node to subscribe to the yellow_lane_angle error and publish error, i_error, d_error

Subscribes to: yellow_lane_angle
msg info:
    std_msgs/Float32

Published to: pid_angle
msg info:
    PIDError
        error       Float32
        i_error     Float32
        d_error     Float32


Mohamed Martini
University of Massachusetts Lowell
"""

import rospy
from std_msgs.msg import Float32
from jetbot_msgs.msg import PIDError
from pid import PID


NODE_NAME = "pid_angle_node"
IN_TOPIC = "yellow_lane_angle"
OUT_TOPIC = "pid_angle"


class PIDAngle:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Float32, self.iterate, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, PIDError, queue_size=1)

        self.pid = PID()

    def iterate(self, msg):
        # receive angle errors
        error_tuple = self.pid.add_error(msg.data)
        self.publish(error_tuple)

    def publish(self, error_tuple):
        # publish angle errors
        msg = PIDError()
        msg.error, msg.i_error, msg.d_error = error_tuple
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)
    PIDAngle()
    rospy.spin()
