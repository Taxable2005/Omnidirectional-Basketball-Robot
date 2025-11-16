#!/usr/bin/env python3

import rospy
import pigpio
from sensor_msgs.msg import Joy
from time import sleep

# Pneumatic GPIO pins
PNEU11 = 6
PNEU12 = 13
PNEU21 = 19
PNEU22 = 26

# Global flag
trigger_dribble = False

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    rospy.logerr("Failed to connect to pigpio daemon. Is it running?")
    exit(1)

# Set all pins as output and low
for pin in [PNEU11, PNEU12, PNEU21, PNEU22]:
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.write(pin, 0)

def run_dribble_sequence():
    rospy.loginfo("Running dribble pneumatic sequence...")

    pi.write(PNEU21, 0)
    pi.write(PNEU22, 1)
    sleep(2)

    pi.write(PNEU12, 0)
    pi.write(PNEU11, 1)
    sleep(1.5)

    pi.write(PNEU21, 0)
    pi.write(PNEU22, 1)
    sleep(1)

    pi.write(PNEU22, 0)
    pi.write(PNEU21, 1)
    sleep(0.8)

    pi.write(PNEU21, 0)
    pi.write(PNEU22, 1)
    sleep(1.5)

    pi.write(PNEU11, 0)
    pi.write(PNEU12, 1)
    sleep(2.5)

    pi.write(PNEU22, 0)
    pi.write(PNEU21, 1)
    sleep(1)

    pi.write(PNEU21, 0)
    pi.write(PNEU22, 1)

    rospy.loginfo("Dribble sequence complete.")

def joy_callback(msg):
    global trigger_dribble
    if msg.buttons[0] == 1:  # X button
        trigger_dribble = True

def main():
    global trigger_dribble

    rospy.init_node("dribble_controller")
    rospy.Subscriber("/joy", Joy, joy_callback)

    rospy.loginfo("Dribble controller node running (pigpio). Press X (button[0]) to activate.")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if trigger_dribble:
            run_dribble_sequence()
            trigger_dribble = False
        rate.sleep()

    pi.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pi.stop()
