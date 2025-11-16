#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float32MultiArray
import math

class ArenaPoseTracker:
    def __init__(self):
        rospy.init_node('arena_pose_tracker')

        self.yaw_deg = 0.0
        self.last_x_robot = None
        self.last_y_robot = None
        self.global_x = 0.0
        self.global_y = 0.0

        rospy.Subscriber('/bno055_data', Float32, self.yaw_callback)
        rospy.Subscriber('/encoder_distance', Float32MultiArray, self.encoder_callback)

        self.coord_pub = rospy.Publisher('/coordinates', Float32MultiArray, queue_size=10)

        self.current_x_robot = 0.0
        self.current_y_robot = 0.0

        rospy.Timer(rospy.Duration(0.05), self.update_global_pose)  # 20Hz

    def yaw_callback(self, msg):
        self.yaw_deg = msg.data

    def encoder_callback(self, msg):
        # Expecting [x, y] in msg.data
        if len(msg.data) >= 2:
            self.current_x_robot = msg.data[0]
            self.current_y_robot = msg.data[1]
        else:
            rospy.logwarn("Received encoder_distance with insufficient data.")

    def update_global_pose(self, event):
        if self.last_x_robot is None or self.last_y_robot is None:
            self.last_x_robot = self.current_x_robot
            self.last_y_robot = self.current_y_robot
            return

        dx_robot = self.current_x_robot - self.last_x_robot
        dy_robot = self.current_y_robot - self.last_y_robot

        self.last_x_robot = self.current_x_robot
        self.last_y_robot = self.current_y_robot

        yaw_rad = math.radians(self.yaw_deg)

        dx_arena = dx_robot * math.cos(yaw_rad) - dy_robot * math.sin(yaw_rad)
        dy_arena = dx_robot * math.sin(yaw_rad) + dy_robot * math.cos(yaw_rad)

        self.global_x += dx_arena
        self.global_y += dy_arena

        # Publish coordinates as Float32MultiArray [x, y]
        coord_msg = Float32MultiArray()
        coord_msg.data = [self.global_x, self.global_y]
        self.coord_pub.publish(coord_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ArenaPoseTracker().run()
