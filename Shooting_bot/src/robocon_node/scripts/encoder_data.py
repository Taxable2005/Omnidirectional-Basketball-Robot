#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Float32MultiArray

# Conversion factor: 1 pulse = 18.84 cm per 10,000 pulses
ENCODER_TO_CM = 18.84 / 10000

def main():
    rospy.init_node('encoder_reader')
    pub = rospy.Publisher('/encoder_distance', Float32MultiArray, queue_size=10)

    ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=0.1)

    rate = rospy.Rate(50)  # 50 Hz
    while not rospy.is_shutdown():
        if ser.in_waiting:
            raw = ser.readline()
            try:
                data = raw.decode('utf-8', errors='ignore').strip()
            except UnicodeDecodeError:
                continue

            if not data:
                continue

            parts = data.split(',')
            if len(parts) == 2 and ':' in parts[0] and ':' in parts[1]:
                try:
                    x_raw = int(parts[0].split(':')[1].strip())
                    y_raw = int(parts[1].split(':')[1].strip())

                    x_cm = x_raw * ENCODER_TO_CM
                    y_cm = y_raw * ENCODER_TO_CM

                    msg = Float32MultiArray()
                    msg.data = [-1*x_cm, y_cm]  # Index 0: X distance, Index 1: Y distance
                    pub.publish(msg)

                    rospy.loginfo(f"Published /encoder_distance: [{x_cm:.2f}, {y_cm:.2f}]")

                except (ValueError, IndexError):
                    continue
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
