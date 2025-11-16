#!/usr/bin/env python3

import rospy
import pigpio
from pigpio_encoder.encoder import Encoder
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from time import sleep

# --- Pin Configuration ---
PUL_PIN = 27
DIR_PIN = 17
ENCODER_A = 14
ENCODER_B = 15

# --- Stepper + Encoder Constants ---
TICKS_PER_23_DEG = 8800.0
DEG_PER_TICK = 23.0 / TICKS_PER_23_DEG   # ~0.00261 deg/tick
STEPS_PER_DEG = 771.70                   # From your stepper code
ANGLE_TOLERANCE = 0.2                    # Acceptable angle error in degrees

# --- Globals ---
target_angle = 0.0
trigger_move = False
pi = pigpio.pi()
encoder = None

def get_current_angle():
    ticks = encoder.get_value()
    angle = ticks * DEG_PER_TICK
    return angle

def move_stepper_to_angle(target_deg):
    curr_angle = get_current_angle()
    error = target_deg - curr_angle

    if abs(error) <= ANGLE_TOLERANCE:
        rospy.loginfo("Target already reached.")
        return

    steps = int(abs(error) * STEPS_PER_DEG)
    direction = pigpio.LOW if error > 0 else pigpio.HIGH
    pi.write(DIR_PIN, direction)

    rospy.loginfo(f"Moving stepper to {target_deg:.2f}°, error: {error:.2f}°, steps: {steps}")

    for _ in range(steps):
        pi.write(PUL_PIN, 1)
        sleep(0.00015)
        pi.write(PUL_PIN, 0)
        sleep(0.00015)

    rospy.loginfo(f"Stepper movement complete. Current angle: {get_current_angle():.2f}°")

def angle_callback(msg):
    global target_angle
    target_angle = msg.data
    rospy.loginfo(f"New target angle set: {target_angle:.2f}°")

def joy_callback(msg):
    global trigger_move
    if msg.buttons[0] == 1:  # X button
        trigger_move = True

def setup_pins():
    pi.set_mode(PUL_PIN, pigpio.OUTPUT)
    pi.set_mode(DIR_PIN, pigpio.OUTPUT)
    pi.write(PUL_PIN, 0)
    pi.write(DIR_PIN, 0)

def main():
    global trigger_move, encoder

    if not pi.connected:
        rospy.logerr("Failed to connect to pigpio daemon!")
        exit(1)

    setup_pins()
    encoder = Encoder(pi, ENCODER_A, ENCODER_B)
    encoder.reset()

    rospy.init_node("stepper_encoder_controller")
    rospy.Subscriber("/angle", Float32, angle_callback)
    rospy.Subscriber("/joy", Joy, joy_callback)

    rospy.loginfo("Stepper controller with encoder feedback running.")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if trigger_move:
            move_stepper_to_angle(target_angle)
            trigger_move = False
        rate.sleep()

    encoder.cancel()
    pi.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        encoder.cancel()
        pi.stop()
