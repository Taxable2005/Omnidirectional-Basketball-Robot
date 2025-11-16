#!/usr/bin/env python3
import rospy
import spidev
from time import sleep

# Initialize ROS node
rospy.init_node("encoder_spi_node")

# Setup SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Use SPI bus 0, device 0 (check wiring)
spi.max_speed_hz = 1000000  # 1 MHz is typical
spi.mode = 0b00  # SPI Mode 0 (CPOL=0, CPHA=0)

rate = rospy.Rate(10)  # 10 Hz loop rate

while not rospy.is_shutdown():
    try:
        # Send dummy bytes and read 4 bytes (2 encoders, 16-bit each)
        data = spi.xfer2([0x00, 0x00, 0x00, 0x00])
        print("Raw SPI Data:", data)

        if len(data) == 4:
            # Combine bytes (assuming little endian: low byte first)
            enc1 = data[1] << 8 | data[0]
            enc2 = data[3] << 8 | data[2]
            print(f"Encoder 1: {enc1}, Encoder 2: {enc2}")
        else:
            print("Invalid response length:", len(data))

    except Exception as e:
        print("SPI read error:", e)

    rate.sleep()
