#!/usr/bin/env python3

import rospy
import serial

def main():
    rospy.init_node('esp32_uart_listener')

    # Connect to UART (ESP32 sends via /dev/ttyAMA0)
    try:
        uart = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
        rospy.loginfo("Listening on /dev/ttyAMA0 at 115200 baud...")
    except serial.SerialException as e:
        rospy.logerr(f"Failed to open serial port: {e}")
        return

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if uart.in_waiting > 0:
            line = uart.readline().decode('utf-8', errors='ignore').strip()
            if line:
                rospy.loginfo(f"Received from ESP32: {line}")
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
