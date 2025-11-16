#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import math

def scan_callback(msg):
    # LaserScan angle range: angle_min to angle_max in radians
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    angle_increment = msg.angle_increment
    ranges = msg.ranges

    # Convert desired angles (degrees) to index
    angle_deg_0 = 0
    angle_deg_90 = 90

    index_0 = int((math.radians(angle_deg_0) - angle_min) / angle_increment)
    index_90 = int((math.radians(angle_deg_90) - angle_min) / angle_increment)

    try:
        dist_0 = ranges[index_0]
        dist_90 = ranges[index_90]
        print("Distance at 0°: {:.2f} m, at 90°: {:.2f} m".format(dist_0, dist_90))
    except IndexError:
        print("Requested angles out of range. Check LIDAR FOV.")

def listener():
    rospy.init_node('scan_angle_reader')
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
