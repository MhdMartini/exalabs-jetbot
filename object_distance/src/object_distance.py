#!/usr/bin/env python3
"""
Node to receive an object's normalized centroid and publish the distance to that object.
If no centroid is found, distance is 1

Example use:
    Subscribes to: red_centroid
        msg info:
                jetbot_msgs/Vector2D
                    float32     x
                    float32     y


    Publishes to: red_distance
        msg info:
            Float32
                data    Float32


Mohamed Martini
University of Massachusetts Lowell
"""
import rospy
from std_msgs.msg import Float32
from jetbot_msgs.msg import Vector2D


class ObjectDistance:
    def __init__(self):
        rospy.Subscriber(IN_TOPIC, Vector2D, self.main, queue_size=1)
        self.pub = rospy.Publisher(OUT_TOPIC, Float32, queue_size=1)

    def calc_distance(self, center_x, center_y):
        # return normalized distance from jetbot's pov (0 to 1)
        return 1 - center_y

    def publish(self, distance):
        self.pub.publish(distance)

    def main(self, msg):
        centroid = msg.x, msg.y
        distance = self.calc_distance(centroid[0], centroid[1]) if centroid != CENTROID_DEF else DIST_DEF
        self.publish(distance)


if __name__ == '__main__':
    NODE_NAME = "object_distance_node"
    rospy.init_node(NODE_NAME)

    IN_TOPIC = "in_topic"
    OUT_TOPIC = "out_topic"
    CENTROID_DEF = (-1, -1)  # centroid received if no object is detected
    DIST_DEF = 1

    ObjectDistance()
    rospy.spin()
