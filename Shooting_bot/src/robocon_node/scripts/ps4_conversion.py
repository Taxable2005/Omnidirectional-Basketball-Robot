#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

pub = None

def joy_callback(data):
    global pub
    vx = -data.axes[0] * 200
    vy = data.axes[1] * 200
    yaw = data.axes[3] * 20

    rospy.loginfo(f"vx: {vx}, vy: {vy}, yaw: {yaw}")

    twist_msg = Twist()
    twist_msg.linear.x = vx
    twist_msg.linear.y = vy
    twist_msg.angular.z = yaw

    pub.publish(twist_msg)

def joy_listener():
    global pub
    rospy.init_node('ps4_joy_listener', anonymous=True)
    pub = rospy.Publisher('/ps4_data', Twist, queue_size=10)
    rospy.Subscriber('/joy', Joy, joy_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        joy_listener()
    except rospy.ROSInterruptException:
        pass
