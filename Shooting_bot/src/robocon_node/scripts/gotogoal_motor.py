#!/usr/bin/env python3

import rospy
import pigpio
from std_msgs.msg import Int32MultiArray, Float32
from sensor_msgs.msg import Joy

# === Motor GPIO Pin Definitions ===
pi = pigpio.pi()
if not pi.connected:
    rospy.logerr("Failed to connect to pigpio daemon!")
    exit()

motors = [
    {"pwm": 20, "dir": 8},   # Motor 1
    {"pwm": 21, "dir": 7},   # Motor 2
    {"pwm": 16, "dir": 12},  # Motor 3
]

# === Initialize Motor Pins ===
for motor in motors:
    pi.set_mode(motor["pwm"], pigpio.OUTPUT)
    pi.set_mode(motor["dir"], pigpio.OUTPUT)

# === Other Actuators ===
pi.set_mode(26, pigpio.OUTPUT)  # Pneumatics PWM
pi.set_mode(19, pigpio.OUTPUT)  # Pneumatics Direction
pi.set_mode(23, pigpio.OUTPUT)  # Shooter 1
pi.set_mode(24, pigpio.OUTPUT)  # Shooter 2

# === Global ===
current_yaw = 0.0

# === Callbacks ===

def imu_callback(data):
    global current_yaw
    current_yaw = data.data

def joy_callback(msg):
    slider = msg.axes[-1]

    # Pneumatic control
    if slider == -1:
        pi.write(19, 1)
        pi.set_PWM_dutycycle(26, 255)
    elif slider == 1:
        pi.write(19, 0)
        pi.set_PWM_dutycycle(26, 255)
    else:
        pi.write(19, 0)
        pi.set_PWM_dutycycle(26, 0)

    # Shooting control
    if msg.buttons[2]:  # Triangle
        pi.write(23, 1)
        pi.write(24, 0)
    elif msg.buttons[0]:  # Cross
        pi.write(23, 0)
        pi.write(24, 1)
    else:
        pi.write(23, 0)
        pi.write(24, 0)

def scale_wheel_speeds_to_pwm(wheel_speeds, pwm_max=255):
    max_speed = max(abs(s) for s in wheel_speeds)
    if max_speed == 0:
        return [{"pwm": 0, "dir": 0} for _ in wheel_speeds]

    scale = pwm_max / max_speed

    pwm_outputs = []
    for speed in wheel_speeds:
        direction = 0 if speed >= 0 else 1
        pwm = int(abs(speed) * scale)
        pwm_outputs.append({"pwm": pwm, "dir": direction})
    
    return pwm_outputs

def motor_callback(msg):
    data = msg.data
    if len(data) != 3:
        rospy.logwarn(f"Expected 3 motor values, got {len(data)}")
        return

    # Use scaled PWM mapping to preserve direction ratios
    wheel_speeds = [float(x) for x in data]
    pwm_outputs = scale_wheel_speeds_to_pwm(wheel_speeds)

    for i in range(3):
        pwm = pwm_outputs[i]["pwm"]
        direction = pwm_outputs[i]["dir"]
        pi.write(motors[i]["dir"], direction)
        pi.set_PWM_dutycycle(motors[i]["pwm"], pwm)

def shutdown_hook():
    rospy.loginfo("Shutting down. Stopping all motors and actuators.")
    stop_cmds = [{"pwm": 0, "dir": 0} for _ in range(3)]
    for i in range(3):
        pi.write(motors[i]["dir"], stop_cmds[i]["dir"])
        pi.set_PWM_dutycycle(motors[i]["pwm"], stop_cmds[i]["pwm"])

    pi.write(19, 0)
    pi.set_PWM_dutycycle(26, 0)
    pi.write(23, 0)
    pi.write(24, 0)
    pi.stop()

# === Main Node ===

def main():
    rospy.init_node('motor_control_node')

    rospy.Subscriber('/motor_value', Int32MultiArray, motor_callback)
    rospy.Subscriber('/joy', Joy, joy_callback)
    rospy.Subscriber('/bno055_data', Float32, imu_callback)

    rospy.on_shutdown(shutdown_hook)

    rospy.loginfo("Motor control node running.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        shutdown_hook()
