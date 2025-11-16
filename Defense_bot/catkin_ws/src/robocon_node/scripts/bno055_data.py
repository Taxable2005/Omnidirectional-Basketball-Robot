#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

import board
import busio
import adafruit_bno055

import math
import time
from collections import deque

def quaternion_to_yaw(w, x, y, z):
    yaw_rad = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    yaw_deg = math.degrees(yaw_rad)
    return yaw_deg

def try_initialize_sensor():
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
        rospy.loginfo("BNO055 connected successfully")
        return sensor
    except Exception as e:
        rospy.logwarn(f"Failed to initialize BNO055: {e}")
        return None

def get_median(data):
    sorted_data = sorted(data)
    n = len(sorted_data)
    if n % 2 == 1:
        return sorted_data[n // 2]
    else:
        return (sorted_data[n // 2 - 1] + sorted_data[n // 2]) / 2.0

def main():
    rospy.init_node('bno055_yaw_publisher', anonymous=True)
    pub = rospy.Publisher('/bno055_data', Float32, queue_size=10)

    sensor = try_initialize_sensor()
    rate = rospy.Rate(50)  # 50 Hz

    alpha = 0.1  # Low-pass filter smoothing factor
    filtered_yaw = None

    median_window = deque(maxlen=5)  # Keep last 5 readings

    while not rospy.is_shutdown():
        if sensor is None:
            sensor = try_initialize_sensor()
            time.sleep(1)
            rate.sleep()
            continue

        try:
            quat = sensor.quaternion
            if quat is None or any(v is None for v in quat):
                rospy.logwarn("Quaternion data unavailable")
            else:
                w, x, y, z = quat
                raw_yaw = quaternion_to_yaw(w, x, y, z)

                # Store in median filter buffer
                median_window.append(raw_yaw)
                median_yaw = get_median(median_window)

                # Apply low-pass filter to median output
                if filtered_yaw is None:
                    filtered_yaw = median_yaw
                else:
                    filtered_yaw = alpha * median_yaw + (1 - alpha) * filtered_yaw

                pub.publish(Float32(data=filtered_yaw))
        except OSError as e:
            rospy.logwarn(f"BNO055 read failed: {e}")
            sensor = None
            time.sleep(1)
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

