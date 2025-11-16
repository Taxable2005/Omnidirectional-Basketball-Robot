#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(scan_msg):
    # Print header
    rospy.loginfo("Received scan at time: %s", scan_msg.header.stamp)

    # Print angle info
    angle_min_deg = scan_msg.angle_min * 180.0 / 3.14159
    angle_max_deg = scan_msg.angle_max * 180.0 / 3.14159
    angle_increment_deg = scan_msg.angle_increment * 180.0 / 3.14159

    rospy.loginfo("Angle Range: %.2f° to %.2f°", angle_min_deg, angle_max_deg)
    rospy.loginfo("Angle Increment: %.2f°", angle_increment_deg)

    # Print distances
    for i, distance in enumerate(scan_msg.ranges):
        angle = angle_min_deg + i * angle_increment_deg
        rospy.loginfo("Angle: %.2f°, Distance: %.2f m", angle, distance)

def main():
    rospy.init_node('ydlidar_scan_printer', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.loginfo("YDLiDAR Scan Printer Node Started")
    rospy.spin()

if __name__ == '__main__':
    main()
