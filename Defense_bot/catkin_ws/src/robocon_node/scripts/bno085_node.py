#!/usr/bin/env python3

import time
import rospy
from geometry_msgs.msg import Quaternion

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C

def main():
    rospy.init_node("bno085_node")
    pub = rospy.Publisher("/bno085_data", Quaternion, queue_size=10)

    # Setup I2C and delay for sensor startup
    i2c = busio.I2C(board.SCL, board.SDA)
    time.sleep(2.0)

    try:
        sensor = BNO08X_I2C(i2c, address=0x4A)
        sensor.enable_feature(sensor.FEATURE_ROTATION_VECTOR)
        rospy.loginfo("✅ BNO085 initialized over I2C")

        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            quat = sensor.quaternion
            if quat:
                msg = Quaternion()
                msg.x = quat[0]
                msg.y = quat[1]
                msg.z = quat[2]
                msg.w = quat[3]
                pub.publish(msg)
            rate.sleep()

    except Exception as e:
        rospy.logerr("❌ Failed to communicate with BNO085: %s", str(e))

if __name__ == "__main__":
    main()

