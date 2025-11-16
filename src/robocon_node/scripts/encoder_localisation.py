#!/usr/bin/env python3

import rospy
import serial
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf

class EncoderOdomPublisher:
    def __init__(self):
        rospy.init_node('encoder_odom_node')
        self.odom_pub = rospy.Publisher('/odom/raw', Odometry, queue_size=10)

        # Update serial port if needed
        self.ser = serial.Serial('/dev/serial0', 115200, timeout=1)

        self.last_x = 0.0
        self.last_y = 0.0
        self.last_time = rospy.Time.now()

    def spin(self):
        rate = rospy.Rate(50)  # Match EKF update rate
        while not rospy.is_shutdown():
            try:
                # Read line, clean up null characters and decode safely
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                line = line.replace('\x00', '')

                if not line:
                    continue

                parts = line.split(',')
                if len(parts) != 2:
                    rospy.logwarn(f"Unexpected format: {line}")
                    continue

                try:
                    x = float(parts[0])
                    y = float(parts[1])
                except ValueError:
                    rospy.logwarn(f"Failed to convert to float: {line}")
                    continue

                self.publish_odom(x, y)

            except Exception as e:
                rospy.logerr(f"Error reading serial data: {e}")

            rate.sleep()

    def publish_odom(self, x, y):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        if dt == 0:
            return

        vx = (x - self.last_x) / dt
        vy = (y - self.last_y) / dt

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(0, 0, 0, 1)  # No orientation from encoders

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0

        # Pose covariance: low values for x and y; high for unused axes
        odom.pose.covariance = [
            0.01, 0,    0,    0,    0,    0,
            0,    0.01, 0,    0,    0,    0,
            0,    0,    9999, 0,    0,    0,
            0,    0,    0,    9999, 0,    0,
            0,    0,    0,    0,    9999, 0,
            0,    0,    0,    0,    0,    9999
        ]

        # Twist covariance: low for linear x/y; high for others
        odom.twist.covariance = [
            0.1,  0,    0,    0,    0,    0,
            0,    0.1,  0,    0,    0,    0,
            0,    0,    9999, 0,    0,    0,
            0,    0,    0,    9999, 0,    0,
            0,    0,    0,    0,    9999, 0,
            0,    0,    0,    0,    0,    9999
        ]

        self.odom_pub.publish(odom)

        self.last_x = x
        self.last_y = y
        self.last_time = current_time


if __name__ == '__main__':
    try:
        node = EncoderOdomPublisher()
        node.spin()
    except rospy.ROSInterruptException:
        pass
