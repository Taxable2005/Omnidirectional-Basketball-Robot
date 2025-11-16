#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Float32MultiArray

def main():
    rospy.init_node('encoder_reader_node')
    pub = rospy.Publisher('/encoder_distance', Float32MultiArray, queue_size=10)

    # Adjust the port name as needed
    ser = serial.Serial('/dev/serial0', 115200, timeout=1)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode().strip()
            if not line:
                continue

            parts = line.split(',')
            if len(parts) != 2:
                rospy.logwarn(f"Unexpected format: {line}")
                continue

            x = float(parts[0])
            y = float(parts[1])

            msg = Float32MultiArray()
            msg.data = [x, y]
            pub.publish(msg)

        except Exception as e:
            rospy.logerr(f"Error reading serial data: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
