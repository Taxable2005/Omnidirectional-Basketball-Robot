#!/usr/bin/env python3

import rospy
import pigpio
from std_msgs.msg import Int32

class SingleEncoderReader:
    def __init__(self):
        rospy.init_node('single_encoder_reader_node')
        self.pub = rospy.Publisher('/encoder_count', Int32, queue_size=10)

        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.logerr("Failed to connect to pigpio daemon")
            exit(1)

        # Define your encoder pins (change as per your wiring)
        self.encoder_A = 16  # Channel A
        self.encoder_B = 20  # Channel B

        self.count = 0

        self.pi.set_mode(self.encoder_A, pigpio.INPUT)
        self.pi.set_mode(self.encoder_B, pigpio.INPUT)

        self.pi.callback(self.encoder_A, pigpio.RISING_EDGE, self.encoder_callback)

    def encoder_callback(self, gpio, level, tick):
        b_state = self.pi.read(self.encoder_B)
        direction = 1 if b_state == 0 else -1
        self.count += direction

    def run(self):
        rate = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():
            msg = Int32()
            msg.data = self.count
            self.pub.publish(msg)
            rate.sleep()

        self.pi.stop()

if __name__ == '__main__':
    try:
        node = SingleEncoderReader()
        node.run()
    except rospy.ROSInterruptException:
        pass
