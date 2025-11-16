#!/usr/bin/env python3

import rospy
import board
import busio
import adafruit_bno08x
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

# === Globals ===
pub = None
yaw_pub = None

# Initialize BNO085
i2c = busio.I2C(board.SCL, board.SDA)
bno = adafruit_bno08x.BNO08X(i2c)

# Enable game rotation vector (quaternion)
bno.enable_feature(adafruit_bno08x.BNO08X_FEATURE_GAME_ROTATION_VECTOR)

def quaternion_to_yaw(w, x, y, z):
    import math
    # Yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    yaw_deg = math.degrees(yaw)
    return yaw_deg

def publish_yaw(event):
    quat = bno.quaternion
    w, x, y, z = quat
    yaw = quaternion_to_yaw(w, x, y, z)
    yaw_pub.publish(yaw)
    rospy.loginfo_throttle(1, f"[BNO085] Yaw: {yaw:.2f} deg")

def ps4_data_callback(data):
    vx_user = data.linear.x
    vy_user = data.linear.y
    rospy.loginfo_throttle(1, f"[PS4] vx: {vx_user:.2f} m/s, vy: {vy_user:.2f} m/s")
    # You can process or republish these values here as needed.

def main():
    global yaw_pub

    rospy.init_node('bno085_data_node')

    yaw_pub = rospy.Publisher('/imu/yaw', Float32, queue_size=10)
    rospy.Subscriber('/joy_vel', Twist, ps4_data_callback)  # adjust topic to your joystick Twist topic

    rospy.loginfo("[BNO085 Node] Started, publishing /imu/yaw at 20 Hz.")

    # Use a timer to publish yaw periodically
    rospy.Timer(rospy.Duration(0.05), publish_yaw)  # 20 Hz

    rospy.spin()

if __name__ == '__main__':
    main()
