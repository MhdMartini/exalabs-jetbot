#!/usr/bin/env python
import sys
import qwiic_scmd
import rospy
from jetbot_msgs.msg import WheelsCommands


class MyWheelsCommands:
    def __init__(self):
        self.rate = rospy.Rate(4)
        if myMotor.connected is False:
            rospy.logwarn("Motor Driver not connected. Check connections.")
            return
        self.init_motor()
        self.set_motors(left=0, right=0)
        myMotor.enable()
        rospy.logwarn("Motor enabled")
        self.rate.sleep()

        rospy.Subscriber(IN_TOPIC, WheelsCommands, self.main, queue_size=10)

    def init_motor(self):
        myMotor.begin()
        rospy.logwarn("Motor initialized.")
        self.rate.sleep()

    def get_dir_val(self, speed):
        # receives speeds from -1 to 1, converts them to direction and value from 0 to 255
        if speed < 0:
            return BWD, speed * -1 * 254
        else:
            return FWD, speed * 254

    def set_motors(self, left, right):
        direction_l, value_l = self.get_dir_val(left)
        myMotor.set_drive(L_MTR, direction_l, value_l)
        direction_r, value_r = self.get_dir_val(right)
        myMotor.set_drive(R_MTR, direction_r, value_r)

    def check_input(self, left, right):
        if not (-1 <= left <= 1):
            rospy.logwarn("Invalid speed value {} for left motor. Speed values should fall between -1 and 1.".format(left))
            left /= abs(left)
        if not (-1 <= right <= 1):
            rospy.logwarn("Invalid speed value {} for right motor. Speed values should fall between -1 and 1.".format(right))
            right /= abs(right)
        return left, right

    def main(self, msg):
        left, right = msg.left, msg.right
        left, right = self.check_input(left, right)
        self.set_motors(left=left, right=right)


def shutdown():
    # Zero Motor Speeds
    myMotor.set_drive(R_MTR, FWD, 0)
    myMotor.set_drive(L_MTR, FWD, 0)
    myMotor.disable()


if __name__ == '__main__':
    myMotor = qwiic_scmd.QwiicScmd()
    R_MTR = 1
    L_MTR = 0
    FWD = 0
    BWD = 1

    NODE_NAME = "wheels_commands_node"
    rospy.init_node(NODE_NAME)

    IN_TOPIC = "in_topic"

    rospy.on_shutdown(shutdown)

    try:
        MyWheelsCommands()
        rospy.spin()
    except (KeyboardInterrupt, SystemExit):
        rospy.logwarn("Stopping Motor!")
        myMotor.disable()
        sys.exit(0)
