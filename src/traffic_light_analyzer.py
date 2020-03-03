#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float32
import message_filters


class traffic_light_analyzer_node:

    def __init__(self):
        # Subscribers
        self.detection_sub = message_filters.Subscriber(
            "/traffic_light_detected", Bool)
        self.size_sub = message_filters.Subscriber(
            "/traffic_light_size", Vector3)

        # Publishers
        self.zone_pub = rospy.Publisher(
            "/zone_height", Float32, queue_size=100)

        # Time synchronizer
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.detection_sub, self.size_sub], 100, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, detection, size):
        # A simple calculation
        if (detection.data == True):
            self.zone_pub.publish(Float32(size.y / 3.0))


def main(args):
    rospy.init_node('traffic_light_analyzer_node', anonymous=True)
    ic = traffic_light_analyzer_node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
