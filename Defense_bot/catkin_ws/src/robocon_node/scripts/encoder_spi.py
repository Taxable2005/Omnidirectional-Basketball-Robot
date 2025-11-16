#!/usr/bin/env python3
import rospy
import spidev
import time
from std_msgs.msg import String

def main():
    spi = spidev.SpiDev()
    spi.open(0, 0)  # Use /dev/spidev0.0
    spi.max_speed_hz = 1000000
    spi.mode = 0b00  # SPI Mode 0

    pub = rospy.Publisher('/encoder_spi', String, queue_size=10)
    rospy.init_node('spi_node', anonymous=True)
    rate = rospy.Rate(5)  # 5 Hz

    rospy.loginfo("Starting SPI communication...")

    try:
        while not rospy.is_shutdown():
            response = spi.xfer2([0xFF] * 5)
            msg = bytes(response).decode('utf-8', errors='ignore').strip()
            rospy.loginfo("Received: %s", msg)
            pub.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        spi.close()
        print("SPI communication stopped.")

if __name__ == '__main__':
    main()
