#!/usr/bin/env python3
import rospy
import smbus2
import struct
from std_msgs.msg import Float32MultiArray

# I2C config
I2C_BUS = 1
ARDUINO_ADDRESS = 0x08
MAX_PACKET_SIZE = 32  # Safety limit

def read_frame(bus):
    try:
        raw = bus.read_i2c_block_data(ARDUINO_ADDRESS, 0, MAX_PACKET_SIZE)

        # Check header
        if raw[0] != 0x06 or raw[1] != 0x85:
            rospy.logwarn("Invalid header")
            return None

        length = raw[2]
        if length != 8:
            rospy.logwarn(f"Unexpected payload length: {length}")
            return None

        payload = raw[3:3+length]
        received_checksum = raw[3+length]

        # Calculate checksum
        checksum = length
        for b in payload:
            checksum ^= b

        if checksum != received_checksum:
            rospy.logwarn("Checksum mismatch")
            return None

        # Unpack payload: 2 floats
        gyro_z, temp = struct.unpack('<ff', bytes(payload))
        return [gyro_z, temp]

    except Exception as e:
        rospy.logerr(f"I2C read error: {e}")
        return None

def main():
    rospy.init_node('adxrs_i2c_receiver')
    pub = rospy.Publisher('/adxrs_data', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    bus = smbus2.SMBus(I2C_BUS)

    rospy.loginfo("ADXRS I2C receiver node started")

    while not rospy.is_shutdown():
        data = read_frame(bus)
        if data:
            msg = Float32MultiArray()
            msg.data = data  # [gyro_z, temp]
            pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
