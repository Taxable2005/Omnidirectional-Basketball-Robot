#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import String

# Open the serial port
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

def read_serial():
    """Main function to read from ESP32"""
    rospy.init_node('esp32_serial_node', anonymous=True)
    pub = rospy.Publisher('/esp32_data', String, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            if line:
                rospy.loginfo(f"Received: {line}")
                pub.publish(line)
        rate.sleep()

if __name__ == '__main__':
    try:
        read_serial()
    except rospy.ROSInterruptException:
        ser.close()
