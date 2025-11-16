#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Int32MultiArray, Float32, Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# === Constants ===
WHEEL_RADIUS = 0.075
ROBOT_RADIUS = 0.2675
IDLE_TIMEOUT = 0.5  # seconds

# PID Constants
Kp = 3.0
Ki = 0.05
Kd = 0.0

# === Globals ===
pub = None
encoder_pub = None
current_yaw = 0.0  # in degrees
vx_user = 0.0
vy_user = 0.0
last_cmd_time = 0.0
last_error = 0.0
integral = 0.0
last_time = None
prevx = 0
prevy = 0
distx = 0.0
disty = 0.0
target = 0.0
rotate = 0

# === Helper Functions ===

def transform_to_robot_frame(vx, vy, yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    vx_r = vx * math.cos(yaw_rad) + vy * math.sin(yaw_rad)
    vy_r = -vx * math.sin(yaw_rad) + vy * math.cos(yaw_rad)
    return vx_r, vy_r

def inverse_kinematics(vx, vy, omega):
    w1 = (-vx / 3) + (vy / math.sqrt(3)) + (omega / 3)
    w2 = (-vx / 3) - (vy / math.sqrt(3)) + (omega / 3)
    w3 = (2 * vx / 3) + (0) + (omega / 3)
    return [w1, w2, w3]

def pid_controller(error, dt):
    global last_error, integral
    integral += error * dt
    derivative = (error - last_error) / dt if dt > 0 else 0.0
    output = Kp * error + Ki * integral + Kd * derivative
    last_error = error
    return output

# === ROS Callback Functions ===

def imu_callback(data):
    global current_yaw
    current_yaw = data.data  # degrees

def ps4_data_callback(data):
    global vx_user, vy_user, target
    vx_user = data.linear.x
    vy_user = data.linear.y
    target = data.angular.z

def joy_callback(data):
    pass

def encoder_callback(data):
    global distx, disty, prevx, prevy, encoder_pub, current_yaw, vx_user, vy_user

    newx = data.data[0]
    newy = data.data[1]
    delx = newx - prevx
    dely = newy - prevy
    prevx = newx
    prevy = newy

    # Convert yaw to radians
    rad = math.radians(current_yaw)

    # Transform to world frame
    dx_world = delx * math.cos(rad) - dely * math.sin(rad)
    dy_world = delx * math.sin(rad) + dely * math.cos(rad)

    # Normalize user command direction
    mag = math.hypot(vx_user, vy_user)
    if mag > 0.01:  # Avoid division by zero and noise
        vx_unit = vx_user / mag
        vy_unit = vy_user / mag

        # Project encoder displacement onto command direction
        dot = dx_world * vx_unit + dy_world * vy_unit
        distx += dot * vx_unit
        disty += dot * vy_unit
    else:
        # No movement command; skip integration
        pass

    # Publish corrected position
    msg = Float32MultiArray()
    msg.data = [distx, disty]
    encoder_pub.publish(msg)

# === Control Loop ===

def control_loop():
    global vx_user, vy_user, current_yaw, last_time, target, rotate

    rate = rospy.Rate(50)  # 50 Hz
    last_time = rospy.get_time()

    while not rospy.is_shutdown():
        now = rospy.get_time()
        dt = now - last_time if last_time is not None else 0.02
        last_time = now
        yaw_error = target - current_yaw
        omega = pid_controller(yaw_error, dt)

        # Transform velocity to robot frame
        vx_r, vy_r = transform_to_robot_frame(vx_user, vy_user, current_yaw)
        wheel_speeds = inverse_kinematics(vx_r, vy_r, omega)

        # Publish motor speeds
        msg = Int32MultiArray()
        msg.data = [int(speed) for speed in wheel_speeds]
        pub.publish(msg)

        rate.sleep()

# === Main ===

def ps4_data_listener():
    global pub, encoder_pub
    rospy.init_node('ps4_data_listener', anonymous=True)

    pub = rospy.Publisher('/motor_value', Int32MultiArray, queue_size=10)
    encoder_pub = rospy.Publisher('/encoder', Float32MultiArray, queue_size=10)

    rospy.Subscriber('/ps4_data', Twist, ps4_data_callback)
    rospy.Subscriber('/bno055_data', Float32, imu_callback)
    rospy.Subscriber('/encoder_distance', Float32MultiArray, encoder_callback)

    control_loop()

if __name__ == '__main__':
    ps4_data_listener()
