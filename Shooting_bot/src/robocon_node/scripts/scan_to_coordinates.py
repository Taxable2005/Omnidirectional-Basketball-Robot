#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
import math

def scan_callback(scan_msg):
    coordinates_pub = rospy.Publisher('/coordinates', PointStamped, queue_size=10)
    angle = scan_msg.angle_min

    for r in scan_msg.ranges:
        if scan_msg.range_min < r < scan_msg.range_max:
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            point = PointStamped()
            point.header = scan_msg.header
            point.point.x = x
            point.point.y = y
            point.point.z = 0

            coordinates_pub.publish(point)
        angle += scan_msg.angle_increment

def main():
    rospy.init_node('scan_to_coordinates')
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
