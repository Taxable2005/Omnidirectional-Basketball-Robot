#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float32  # Message type for speed control

# Define GPIO pins for L298N Motor Driver
ENA = 18  # PWM Pin
IN1 = 17  # Motor Direction Pin 1
IN2 = 27  # Motor Direction Pin 2

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# Setup PWM
pwm = GPIO.PWM(ENA, 1000)  # Set PWM frequency to 1 kHz
pwm.start(0)  # Start with 0% duty cycle (motor off)

def motor_callback(speed_msg):
    """Callback function to control motor speed and direction."""
    speed = speed_msg.data  # Extract speed value from the ROS message

    if speed > 0:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    elif speed < 0:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)

    pwm.ChangeDutyCycle(abs(speed))  # Set motor speed

def motor_controller():
    """ROS Node to listen for speed commands and control the motor."""
    rospy.init_node('motor_controller', anonymous=True)
    rospy.Subscriber('/motor_speed', Float32, motor_callback)
    rospy.spin()
    GPIO.cleanup()

if __name__ == '__main__':
    try:
        motor_controller()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
