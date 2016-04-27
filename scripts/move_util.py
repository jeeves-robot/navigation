import rospy
from geometry_msgs.msg import Twist
import time

### Note: This file provides a variety of different movement
### modes that the robot can take.

# Some frequently used twist commands we will want to publish

NAV_TOPIC = '/cmd_vel_mux/input/navi'


def twist_forward():
    twistForward = Twist()
    twistForward.linear.x = 0.05
    return twistForward


def twist_backward():
    twistBackward = Twist()
    twistBackward.linear.x = -0.05
    return twistBackward


def twist_right():
    twistRight = Twist()
    twistRight.angular.z = -3.14
    return twistRight


def twist_left():
    twistLeft = Twist()
    twistLeft.angular.z = 3.14


def forward_back(pub):
    pub.publish(twist_forward())
    time.sleep(0.2)
    pub.publish(twist_backward())
    time.sleep(0.2)


def twist_direction(pub, direction):
    pub.publish(direction)
    time.sleep(0.5)