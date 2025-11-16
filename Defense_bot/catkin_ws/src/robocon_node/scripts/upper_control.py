#!/usr/bin/env python3

import rospy
import pigpio
import math
import threading
import serial
from std_msgs.msg import Int32, Float32MultiArray, Float32
from sensor_msgs.msg import Joy
from time import sleep

# === Constants ===
ESC_GPIO_PIN = 4
MIN_THROTTLE = 1000
MAX_THROTTLE = 2000

# Pneumatic GPIO pins
PNEU10 = 13
PNEU11 = 19
PNEU20 = 26
PNEU21 = 17
PNEU30 = 6
PNEU31 = 5


# === Global States ===
pi = pigpio.pi()
if not pi.connected:
    rospy.logerr("Could not connect to pigpio daemon. Exiting.")
    exit(1)

# UART setup
try:
    uart = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
    rospy.loginfo("UART serial port opened.")
except serial.SerialException as e:
    rospy.logerr(f"Failed to open UART: {e}")
    uart = None

current_throttle = MIN_THROTTLE
motor_enabled = False
last_button_state = 0
last_square_state = 0
trigger_dribble = False

target = 0.0

shooting_angle_pub = None

# === Init GPIOs ===
for pin in [PNEU10, PNEU11, PNEU20, PNEU21, PNEU30, PNEU31]:
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.write(pin, 0)

# R1 GPIO setup

# === ESC PWM Control ===
def set_esc_pwm(microseconds):
    if microseconds == 0:
        pi.set_servo_pulsewidth(ESC_GPIO_PIN, 0)
        rospy.loginfo("PWM disabled (0 µs)")
    elif MIN_THROTTLE <= microseconds <= MAX_THROTTLE:
        pi.set_servo_pulsewidth(ESC_GPIO_PIN, microseconds)
        rospy.loginfo(f"Throttle: {microseconds} µs")
    else:
        rospy.logwarn("Invalid throttle input")

# === Throttle Callback ===
def throttle_callback(msg):
    global current_throttle, motor_enabled
    if MIN_THROTTLE <= msg.data <= MAX_THROTTLE:
        current_throttle = msg.data
        if motor_enabled:
            set_esc_pwm(current_throttle)
    else:
        rospy.logwarn(f"Invalid throttle: {msg.data}")

# === Pneumatic Dribble Sequence ===
def run_dribble_sequence():
    rospy.loginfo("Running dribble pneumatic sequence...")

    # Send hello over UART
    if uart:
        try:
            uart.write(b'hello\n')
            rospy.loginfo("Sent 'hello' over UART to ESP32")
        except serial.SerialException as e:
            rospy.logwarn(f"UART write failed: {e}")
    else:
        rospy.logwarn("UART not initialized; skipping UART send")
        rospy.loginfo("Dribble sequence complete.")

# === PS4 Joy Callback ===
def joy_callback(msg):
    global motor_enabled, last_button_state, last_square_state, target, m, X_cord, Y_cord, trigger_dribble

    square = msg.buttons[3]
    x_button = msg.buttons[0]
    r2 = msg.axes[5]

    r2 = (1 - r2) * 500 + 1000

    global current_throttle
    if MIN_THROTTLE <= r2 <= MAX_THROTTLE:
        current_throttle = r2
        if motor_enabled:
            set_esc_pwm(current_throttle)

    if square == 1 and last_square_state == 0:
        trigger_dribble = True

    if x_button == 1 and last_button_state == 0:
        motor_enabled = not motor_enabled
        if motor_enabled:
            rospy.loginfo("X pressed: Motor ENABLED")
            set_esc_pwm(current_throttle)
        else:
            rospy.loginfo("X pressed: Motor DISABLED")
            set_esc_pwm(MIN_THROTTLE)


    last_button_state = x_button
    last_square_state = square

# === Main Node ===
def main():
    global trigger_dribble

    rospy.init_node("esc_dribble_combined_node")

    rospy.Subscriber("/joy", Joy, joy_callback)
    rospy.Subscriber("/esc_throttle", Int32, throttle_callback)

    rospy.loginfo("Combined ESC + Dribble controller node started.")
    rospy.loginfo("Sending initial 1000 µs to arm ESC...")
    set_esc_pwm(MIN_THROTTLE)
    rospy.sleep(5.0)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if trigger_dribble:
            threading.Thread(target=run_dribble_sequence, daemon=True).start()
            trigger_dribble = False
        rate.sleep()

    rospy.loginfo("Shutting down, disabling PWM.")
    set_esc_pwm(0)
    pi.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pi.stop()

