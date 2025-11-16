#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32, Float32MultiArray, Int32MultiArray

# === Constants ===
WHEEL_RADIUS = 0.075
ROBOT_RADIUS = 0.2675

# PID constants
Kp_lin = 2.0
Ki_lin = 0.0
Kd_lin = 0.1

Kp_ang = 3.0
Ki_ang = 0.05
Kd_ang = 0.0

# === Globals ===
current_pos = [0.0, 0.0]
target_pos = [0.0, 0.0]
current_yaw = 0.0

last_time = None
last_error_lin = 0.0
integral_lin = 0.0
last_error_ang = 0.0
integral_ang = 0.0

motor_pub = None

# === Helper Functions ===

def transform_to_robot_frame(vx, vy, yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    vx_r = vx * math.cos(yaw_rad) + vy * math.sin(yaw_rad)
    vy_r = -vx * math.sin(yaw_rad) + vy * math.cos(yaw_rad)
    return vx_r, vy_r

def inverse_kinematics(vx, vy, omega):
    w1 = (-vx / 3.0) + (vy / math.sqrt(3)) + (omega / 3.0)
    w2 = (-vx / 3.0) - (vy / math.sqrt(3)) + (omega / 3.0)
    w3 = (2 * vx / 3.0) + 0 + (omega / 3.0)
    return [w1, w2, w3]

def pid_controller(error, dt, last_error, integral, Kp, Ki, Kd):
    integral += error * dt
    derivative = (error - last_error) / dt if dt > 0 else 0
    output = Kp * error + Ki * integral + Kd * derivative
    return output, integral, error

def scale_wheel_speeds(wheel_speeds, pwm_max=255):
    max_speed = max(abs(s) for s in wheel_speeds)
    if max_speed == 0:
        return [0, 0, 0]
    scale = pwm_max / max_speed
    scaled = [int(s * scale) for s in wheel_speeds]
    return scaled

# === Callbacks ===

def imu_callback(data):
    global current_yaw
    current_yaw = data.data

def encoder_callback(data):
    global current_pos
    current_pos = data.data[:2]

def target_callback(data):
    global target_pos
    target_pos = data.data[:2]

# === Main Control Loop ===

def control_loop():
    global last_time, last_error_lin, integral_lin, last_error_ang, integral_ang

    rate = rospy.Rate(50)
    last_time = rospy.get_time()

    while not rospy.is_shutdown():
        now = rospy.get_time()
        dt = now - last_time
        last_time = now

        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        distance = math.hypot(dx, dy)

        # Linear velocity
        vx_user = dx / distance if distance > 0.01 else 0
        vy_user = dy / distance if distance > 0.01 else 0

        linear_speed, integral_lin, last_error_lin = pid_controller(
            distance, dt, last_error_lin, integral_lin, Kp_lin, Ki_lin, Kd_lin
        )

        vx = vx_user * linear_speed
        vy = vy_user * linear_speed

        # Angular correction
        desired_angle = math.degrees(math.atan2(dy, dx))
        angle_error = desired_angle - current_yaw
        angle_error = (angle_error + 180) % 360 - 180

        omega, integral_ang, last_error_ang = pid_controller(
            angle_error, dt, last_error_ang, integral_ang, Kp_ang, Ki_ang, Kd_ang
        )

        # Transform and compute wheel speeds
        vx_r, vy_r = transform_to_robot_frame(vx, vy, current_yaw)
        raw_speeds = inverse_kinematics(vx_r, vy_r, omega)

        # Scale speeds proportionally and maintain sign
        scaled_pwm = scale_wheel_speeds(raw_speeds)
        signed_pwm = [
            pwm if speed >= 0 else -pwm
            for pwm, speed in zip(scaled_pwm, raw_speeds)
        ]

        msg = Int32MultiArray()
        msg.data = signed_pwm
        motor_pub.publish(msg)

        rate.sleep()

# === Main ===

def go_to_goal_node():
    global motor_pub
    rospy.init_node('go_to_goal_node')

    motor_pub = rospy.Publisher('/motor_value', Int32MultiArray, queue_size=10)

    rospy.Subscriber('/encoder_distance', Float32MultiArray, encoder_callback)
    rospy.Subscriber('/bno055_data', Float32, imu_callback)
    rospy.Subscriber('/target', Float32MultiArray, target_callback)

    rospy.loginfo("Go-to-goal node started.")
    control_loop()

if __name__ == '__main__':
    try:
        go_to_goal_node()
    except rospy.ROSInterruptException:
        pass

