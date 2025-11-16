#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import PointStamped
from tf.transformations import euler_from_quaternion
import math


class PositionEstimator:
    def __init__(self):
        rospy.init_node('position_node')

        self.front_distance = None
        self.left_distance = None
        self.yaw = None

        # Arena size in meters
        self.arena_length = 15.0  # X direction
        self.arena_width = 8.0    # Y direction

        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/bno055_data', Imu, self.imu_callback)

        # Publisher
        self.position_pub = rospy.Publisher('/position', PointStamped, queue_size=10)

        rospy.Timer(rospy.Duration(0.2), self.compute_position)

    def scan_callback(self, msg):
        try:
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment

            index_front = int((0.0 - angle_min) / angle_increment)
            index_left = int((math.pi/2 - angle_min) / angle_increment)

            self.front_distance = msg.ranges[index_front] if index_front < len(msg.ranges) else None
            self.left_distance = msg.ranges[index_left] if index_left < len(msg.ranges) else None

        except Exception as e:
            rospy.logwarn("Scan parsing failed: %s", e)

    def imu_callback(self, msg):
        try:
            orientation_q = msg.orientation
            q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (_, _, yaw) = euler_from_quaternion(q)
            self.yaw = yaw
        except Exception as e:
            rospy.logwarn("IMU parsing failed: %s", e)

    def compute_position(self, event):
        print("📡 Timer triggered")
        print("  ➤ front:", self.front_distance)
        print("  ➤ left :", self.left_distance)
        print("  ➤ yaw  :", self.yaw)

        if self.front_distance is None or self.left_distance is None or self.yaw is None:
            print("⚠️ Missing data — skipping computation.")
            return

        # Adjust position based on orientation
        x = self.arena_length - self.front_distance * math.cos(self.yaw)
        y = self.arena_width - self.left_distance * math.sin(self.yaw)

        position_msg = PointStamped()
        position_msg.header.stamp = rospy.Time.now()
        position_msg.header.frame_id = "map"
        position_msg.point.x = x
        position_msg.point.y = y
        position_msg.point.z = 0.0

        self.position_pub.publish(position_msg)
        rospy.loginfo("📍 Estimated position: x=%.2f, y=%.2f, yaw=%.2f°", x, y, math.degrees(self.yaw))


if __name__ == '__main__':
    try:
        PositionEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
