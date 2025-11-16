#!/usr/bin/env python3

import rospy
import pigpio
from std_msgs.msg import Int32

ENCODER_A = 17  # GPIO 17 for Channel A
ENCODER_B = 27  # GPIO 27 for Channel B

class EncoderReader:
    def __init__(self):
        rospy.init_node("encoder_publisher", anonymous=True)
        self.pub = rospy.Publisher("/encoder_data", Int32, queue_size=10)

        self.pi = pigpio.pi()
        self.pi.set_mode(ENCODER_A, pigpio.INPUT)
        self.pi.set_mode(ENCODER_B, pigpio.INPUT)

        self.pi.set_pull_up_down(ENCODER_A, pigpio.PUD_UP)
        self.pi.set_pull_up_down(ENCODER_B, pigpio.PUD_UP)

        self.last_A = self.pi.read(ENCODER_A)
        self.last_B = self.pi.read(ENCODER_B)

        self.position = 0
        self.pi.callback(ENCODER_A, pigpio.EITHER_EDGE, self.update)
        self.pi.callback(ENCODER_B, pigpio.EITHER_EDGE, self.update)

    def update(self, gpio, level, tick):
        A = self.pi.read(ENCODER_A)
        B = self.pi.read(ENCODER_B)

        if A == self.last_A and B == self.last_B:
            return  # No change

        if A != self.last_A:
            if A == B:
                self.position += 1
            else:
                self.position -= 1
        else:
            if A == B:
                self.position -= 1
            else:
                self.position += 1

        self.last_A = A
        self.last_B = B

        self.pub.publish(self.position)
        rospy.loginfo(f"Encoder Position: {self.position}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        encoder = EncoderReader()
        encoder.run()
    except rospy.ROSInterruptException:
        pass
