#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import sys
sys.path.append('/home/innovation/robocon/src/robocon_node/scripts')
from ydlidar_x2 import YDLidarX2
import math
import time


class LidarPublisher:
    def __init__(self):
        rospy.init_node('ydlidar_x2_node')

        self.lidar = YDLidarX2("/dev/serial0")
        self.lidar.baudrate = 128000

        if not self.lidar.connect():
            rospy.logerr("❌ Could not connect to YDLIDAR X2")
            exit(1)

        rospy.loginfo("✅ LiDAR connected and scan started")
        self.lidar.start_scan()
        time.sleep(1.0)

        self.publisher = rospy.Publisher('scan', LaserScan, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

    def publish_loop(self):
        while not rospy.is_shutdown():
            rospy.loginfo("🌀 Loop running")

            if not self.lidar.available:
                rospy.logwarn("⚠️ LiDAR not available")
                self.rate.sleep()
                continue

            distances = self.lidar.get_data()
            rospy.loginfo("📊 Got distances: %s", distances[:5])  # show first 5

            msg = LaserScan()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'laser_frame'

            msg.angle_min = 0.0
            msg.angle_max = 2 * math.pi
            msg.angle_increment = 2 * math.pi / 360.0
            msg.time_increment = 0.0
            msg.scan_time = 0.1
            msg.range_min = 0.4
            msg.range_max = 15.0

            msg.ranges = [
                float(d) / 1000.0 if 400 <= d <= 15000 else float('inf')
                for d in distances
            ]

            msg.intensities = [100.0] * len(msg.ranges)
            self.publisher.publish(msg)

            self.rate.sleep()

    def stop(self):
        self.lidar.stop()
        self.lidar.disconnect()
        rospy.loginfo("🛑 LiDAR stopped and disconnected")


if __name__ == '__main__':
    try:
        node = LidarPublisher()
        node.publish_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.stop()
