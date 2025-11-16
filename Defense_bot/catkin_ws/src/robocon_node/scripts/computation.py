#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Int32MultiArray, Float32, Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy  # For joy message

# === Constants ===
WHEEL_RADIUS = 0.075
ROBOT_RADIUS = 0.2675
IDLE_TIMEOUT = 0.5  # seconds

# === PID Constants (tune these) ===
Kp = 1.0
Ki = 0.0
Kd = 0.0

# === Globals ===
pub = None
current_yaw = 0.0
vx_user = 0.0
vy_user = 0.0
last_cmd_time = 0.0
last_error = 0.0
integral = 0.0
last_time = None

target = 0.0

# === Callback Functions ===
def transform_to_robot_frame(vx, vy, yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    vx_r = vx * math.cos(yaw_rad) + vy * math.sin(yaw_rad)
    vy_r = -vx * math.sin(yaw_rad) + vy * math.cos(yaw_rad)
    return vx_r, vy_r

def inverse_kinematics(vx, vy, omega):
    L = ROBOT_RADIUS
    R = WHEEL_RADIUS
    w1 = 1.3 * (0.3536 * (-vx + vy)) + (0.09 / R) * omega
    w2 = 1.3 * (0.3536 * (-vx - vy)) + (0.09 / R) * omega
    w3 = 1.3 * (0.3536 * ( vx - vy)) + (0.09 / R) * omega
    w4 = 1.3 * (0.3536 * ( vx + vy)) + (0.09 / R) * omega
    return [w1, w2, w3, w4]

def imu_callback(data):
    global current_yaw
    current_yaw = data.data

def ps4_data_callback(data):
    global vx_user, vy_user, target
    vx_user = data.linear.x
    vy_user = data.linear.y
    target = data.angular.z

def pid_controller(error, dt):
    global last_error, integral
    integral += error * dt
    derivative = (error - last_error) / dt if dt > 0 else 0.0
    output = Kp * error + Ki * integral + Kd * derivative
    last_error = error
    return output

def control_loop():
    global vx_user, vy_user, current_yaw, last_time, target

    rate = rospy.Rate(50)  # 50 Hz
    last_time = rospy.get_time()

    while not rospy.is_shutdown():
        now = rospy.get_time()
        dt = now - last_time if last_time is not None else 0.02
        last_time = now

        yaw_error = target - current_yaw
        omega = pid_controller(yaw_error, dt)

        vx = vx_user
        vy = vy_user
        vx_r, vy_r = transform_to_robot_frame(vx, vy, current_yaw)
        wheel_speeds = inverse_kinematics(vx_r, vy_r, omega)

        msg = Int32MultiArray()
        msg.data = [int(speed) for speed in wheel_speeds]
        pub.publish(msg)

        rate.sleep()

def ps4_data_listener():
    global pub
    rospy.init_node('ps4_data_listener', anonymous=True)

    pub = rospy.Publisher('/motor_value', Int32MultiArray, queue_size=10)

    rospy.Subscriber('/ps4_data', Twist, ps4_data_callback)
    rospy.Subscriber('/bno055_data', Float32, imu_callback)

    control_loop()

if __name__ == '__main__':
    ps4_data_listener()
