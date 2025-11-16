#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math

def imu_callback(msg):
    # Get quaternion from IMU message
    q = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )
    
    # Convert quaternion to Euler angles (roll, pitch, yaw) in radians
    roll, pitch, yaw = euler_from_quaternion(q)
    
    # Convert yaw from radians to degrees
    yaw_deg = math.degrees(yaw)
    
    # By default, yaw increases counterclockwise in standard ENU frame,
    # but sometimes coordinate frame differs. Adjust sign if needed.
    # If clockwise rotation gives positive instead, just do: yaw_deg = -yaw_deg
    
    rospy.loginfo(f"Yaw: {yaw_deg:.2f} degrees")

def listener():
    rospy.init_node('bno_yaw_listener', anonymous=True)
    rospy.Subscriber("/data", Imu, imu_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
