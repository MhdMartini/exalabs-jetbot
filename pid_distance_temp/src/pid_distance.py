#!/usr/bin/env python3
"""
Node to subscribe to the vehicle_distance error and publish error, i_error, d_error

Subscribes to: vehicle_distance
msg info:
    std_msgs/Float32

Published to: pid_distance
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


NODE_NAME = "pid_distance_node"
IN_TOPIC = "vehicle_distance"
OUT_TOPIC = "pid_distance"

PARAM_DIST = "DESIRED_DISTANCE"
DESIRED_DIST = 0.2


class PIDDistance:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Float32, self.iterate, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, PIDError, queue_size=1)

        self.pid = PID()

    def iterate(self, msg):
        # receive distance errors
        desired = rospy.get_param(PARAM_DIST)
        try:
            assert 0 <= desired <= 1
        except AssertionError as e:
            rate = rospy.Rate(2)
            rospy.logwarn(e)
            rate.sleep()
            return

        error_tuple = self.pid.add_error(msg.data - desired)
        self.publish(error_tuple)

    def publish(self, error_tuple):
        # publish distance errors
        msg = PIDError()
        msg.error, msg.i_error, msg.d_error = error_tuple
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.set_param(PARAM_DIST, DESIRED_DIST)
    PIDDistance()
    rospy.spin()
