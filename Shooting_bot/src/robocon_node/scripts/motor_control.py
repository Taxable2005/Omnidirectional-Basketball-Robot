#!/usr/bin/env python3
import rospy
import pigpio
from std_msgs.msg import Int32MultiArray, Float32
from sensor_msgs.msg import Joy

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    rospy.logerr("Failed to connect to pigpio daemon!")
    exit(1)

# Motor GPIO pins: PWM and Direction
motors = [
    {"pwm": 21, "dir": 20},  # Motor 1
    {"pwm": 16, "dir": 12},  # Motor 2
    {"pwm": 13, "dir": 6},   # Motor 3
]

# Setup GPIO pins for motors
for motor in motors:
    pi.set_mode(motor["pwm"], pigpio.OUTPUT)
    pi.set_mode(motor["dir"], pigpio.OUTPUT)

# Additional outputs
pi.set_mode(26, pigpio.OUTPUT)
pi.set_mode(19, pigpio.OUTPUT)
pi.set_mode(23, pigpio.OUTPUT)
pi.set_mode(24, pigpio.OUTPUT)

# Global to store latest yaw
current_yaw = 0.0

def imu_callback(data):
    global current_yaw
    current_yaw = data.data

def control_motors(motor_commands):
    """
    Sets motor speed and direction.
    motor_commands: list of dicts with keys 'pwm' (0-255) and 'dir' (0 or 1)
    """
    for i, command in enumerate(motor_commands):
        motor = motors[i]
        pi.write(motor["dir"], command["dir"])
        pi.set_PWM_dutycycle(motor["pwm"], command["pwm"])

def slider_callback(msg):
    slider = msg.axes[-1]

    # Pneumatics control
    if slider == -1:
        pi.write(19, 1)
        pi.set_PWM_dutycycle(26, 255)
    elif slider == 1:
        pi.write(19, 0)
        pi.set_PWM_dutycycle(26, 255)
    else:
        pi.write(19, 0)
        pi.set_PWM_dutycycle(26, 0)

    # Shoot mechanism
    if msg.buttons[2]:  # Triangle
        pi.write(23, 1)
        pi.write(24, 0)
    elif msg.buttons[0]:  # Cross
        pi.write(23, 0)
        pi.write(24, 1)
    else:
        pi.write(23, 0)
        pi.write(24, 0)

def callback(msg):
    data = msg.data

    if len(data) != 3:
        rospy.logwarn(f"Expected 3 elements in /motor_value, got {len(data)}")
        return

    motor_commands = []
    signed_pwms = []
    for pwm in data:
        direction = 0 if pwm >= 0 else 1
        pwm_val = max(0, min(255, abs(pwm)))
        signed_pwm = pwm_val if direction == 0 else -pwm_val
        signed_pwms.append(signed_pwm)
        motor_commands.append({"pwm": pwm_val, "dir": direction})

    control_motors(motor_commands)
    # rospy.loginfo(f"Yaw: {current_yaw:.2f}°, Motor PWM values: {signed_pwms}")

def shutdown_hook():
    rospy.loginfo("Shutting down: stopping all motors.")
    stop_cmds = [{"pwm": 0, "dir": 0} for _ in range(3)]
    control_motors(stop_cmds)
    pi.write(19, 0)
    pi.set_PWM_dutycycle(26, 0)
    pi.write(23, 0)
    pi.write(24, 0)
    pi.stop()

def main():
    rospy.init_node('motor_control_node')
    rospy.Subscriber('/joy', Joy, slider_callback)
    rospy.Subscriber('/motor_value', Int32MultiArray, callback)
    rospy.Subscriber('/bno055_data', Float32, imu_callback)
    rospy.on_shutdown(shutdown_hook)

    rospy.loginfo("Motor control node running.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        shutdown_hook()
