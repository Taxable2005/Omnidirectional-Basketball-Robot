#!/usr/bin/env python3
import rospy
import pigpio
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy  # Import Joy message type

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    rospy.logerr("Failed to connect to pigpio daemon!")
    exit(1)

# Motor GPIO pins: PWM and Direction
motors = [
    {"pwm": 16, "dir": 12},   # Motor 1
    {"pwm": 20, "dir": 21},  # Motor 2
    {"pwm": 25, "dir": 23},  # Motor 3
    {"pwm": 8, "dir": 7},   # Motor 4
]

# Auxiliary motor on GPIO 18 (no direction pin)
AUX_MOTOR_PWM = 18
pi.set_mode(AUX_MOTOR_PWM, pigpio.OUTPUT)

# Setup main motor GPIOs
for motor in motors:
    pi.set_mode(motor["pwm"], pigpio.OUTPUT)
    pi.set_mode(motor["dir"], pigpio.OUTPUT)
    rospy.loginfo(f"Set GPIO {motor['pwm']} and {motor['dir']} to OUTPUT mode")

def control_motors(motor_commands):
    """
    Sets motor speed and direction.
    motor_commands: list of dicts with keys 'pwm' (0-255) and 'dir' (0 or 1)
    """
    for i, command in enumerate(motor_commands):
       
         motor = motors[i]
         pi.write(motor["dir"], command["dir"])
         pi.set_PWM_dutycycle(motor["pwm"], 0.7*command["pwm"])
         #pi.write(motor["dir"], 1)
         #pi.set_PWM_dutycycle(motor["pwm"], 100)
def motor_callback(msg):
    data = msg.data

    if len(data) != 4:
        rospy.logwarn(f"[motor_control_node] Expected 4 values in /motor_value, got {len(data)}")
        return

    motor_commands = []
    for pwm in data:
        direction = 0 if pwm >= 0 else 1
        pwm_val = max(0, min(255, abs(pwm)))  # Clamp to [0, 255]
        motor_commands.append({"pwm": pwm_val, "dir": direction})
    control_motors(motor_commands)
    rospy.loginfo(f"[motor_control_node] Motor commands: {motor_commands}")

def joy_callback(msg):
    """
    Run motor on GPIO 18 at full speed (PWM 255) while button 3 is pressed.
    Stop immediately when the button is released.
    """
    if len(msg.buttons) > 3:
        if msg.buttons[4] == 1:  # Button is pressed
            pi.set_PWM_dutycycle(AUX_MOTOR_PWM, 255)
            rospy.loginfo("Auxiliary motor ON (PWM 255)")
        else:  # Button is not pressed
            pi.set_PWM_dutycycle(AUX_MOTOR_PWM, 0)
            rospy.loginfo("Auxiliary motor OFF (PWM 0)")

def shutdown_hook():
    rospy.loginfo("[motor_control_node] Shutting down: stopping all motors.")
    stop_cmds = [{"pwm": 0, "dir": 0} for _ in range(4)]
    control_motors(stop_cmds)
    pi.set_PWM_dutycycle(AUX_MOTOR_PWM, 0)  # Stop auxiliary motor
    pi.stop()

def main():
    rospy.init_node('motor_control_node')
    rospy.Subscriber('/motor_value', Int32MultiArray, motor_callback)
    rospy.Subscriber('/joy', Joy, joy_callback)  # Subscribe to /joy topic
    rospy.on_shutdown(shutdown_hook)

    rospy.loginfo("[motor_control_node] Node running. Listening on /motor_value and /joy...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        shutdown_hook()
