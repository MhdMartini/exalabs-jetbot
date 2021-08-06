#!/usr/bin/env python3
"""
Node to subscribe to the yellow_lane_angle error and publish error, i_error, d_error

Subscribes to: error_topic
msg info:
    std_msgs/Float32

Published to: pid_errors
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


NODE_NAME = "pid_errors_node"
IN_TOPIC = "error_topic"
OUT_TOPIC = "pid_errors"

PARAM_SMART_ERROR = "SMART_ERROR"
PARAM_SMART_ERROR_DEF = 0

# if error is zero and past error was <= 0.6, make current error = past_error. Only if SMART_ERROR = 1
PARAM_LIMIT = "LIMIT"
PARAM_LIMIT_DEF = 0.6


class PIDErrors:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Float32, self.main, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, PIDError, queue_size=1)

        self.pid = PID()
        self.past_error = 0

    def publish(self, error_tuple):
        # publish angle errors
        msg = PIDError()
        msg.error, msg.i_error, msg.d_error = error_tuple
        self.pub.publish(msg)

    def get_error(self, error):
        return error

    def main(self, msg):
        error = self.get_error(msg.data)
        error_tuple = self.pid.add_error(error)
        self.publish(error_tuple)


class PIDErrorsSmart(PIDErrors):
    def get_error(self, error):
        if error == 0:
            # check if it is really zero
            if abs(self.past_error) >= rospy.get_param(PARAM_LIMIT, PARAM_LIMIT_DEF):
                # error = self.past_error
                return self.past_error
        self.past_error = error
        return error


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)
    SMART = rospy.get_param(PARAM_SMART_ERROR, PARAM_SMART_ERROR_DEF)
    if SMART:
        PIDErrorsSmart()
    else:
        PIDErrors()
    rospy.spin()
