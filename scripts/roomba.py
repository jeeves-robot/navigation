#!/usr/bin/env python

# Note: Need to source devel/setup.bash for this directory to get
# the package to be recognized

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import MotorPower
import random
import time 

topic = '/cmd_vel_mux/input/navi'

bumped = False
pub = rospy.Publisher(topic, Twist, queue_size=5)

# This method is called when we hit something.
# Randomly choose a direction
def changeDirection(bump):
    global bumped
    bumped = True

def rotate():
    global bumped
    twist = Twist()

    # The TA told me to do something like this, but it's not quite working.
    # She basically said to keep calling rotate 10 times for a second or two. 
    for i in range(15):
        twist.linear.x = -0.05
        twist.angular.x = 0
        pub.publish(twist)
        time.sleep(.02)
    angle = random.uniform(-2, 2)
    for i in range(50):
        twist.linear.x = -0.05
        twist.angular.z = 2
        pub.publish(twist)
        time.sleep(.02)
    pub.publish(twist)
    bumped = False

def move_forward():
    twist = Twist()
    twist.linear.x = 0.05
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub.publish(twist)
    time.sleep(0.2)

# Rotate back and forth to show that it is stuck.
# Steps to stuck dance:
#   Forward and back
#   Turn 90 degrees right, forward back
#   Turn 90 degrees left, forward, back, and repeat
#   Turn 90 degrees to face in the original direction
# If problem persists, we can repeat
def stuck_dance():
    twistForward = Twist()
    twistForward.linear.x = 0.05

    twistBackward = Twist()
    twistBackward.linear.x = -0.05

    # The units for angular.z are radians/second
    # Assumes that after this command will sleep for .5 seconds
    # Will turn (about) 90 degrees
    twistRight = Twist()
    twistRight.angular.z = 3.14

    twistLeft = Twist()
    twistLeft.angular.z = -1 * twistRight.angular.z

    def forward_back():
        twistForward = Twist()
        twistForward.linear.x = 0.05

        twistBackward = Twist()
        twistBackward.linear.x = -0.05

        pub.publish(twistForward)
        time.sleep(0.2)

        pub.publish(twistBackward)
        time.sleep(0.2)

    def twist_direction(direction):
        pub.publish(direction)
        time.sleep(0.5)

    forward_back()
    twist_direction(twistRight)
    forward_back()

    twist_direction(twistLeft)
    forward_back()
    twist_direction(twistLeft)
    forward_back()
    twist_direction(twistRight)

if __name__ == "__main__":
    rospy.init_node('turtlebot_roomba')
 
    # Subscribe to topic /mobile_base/events/bumper
    # to get notifications when we bump into things.
    sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, changeDirection)

    try: 
        while(1):
            if bumped is True:
                rotate()
            else:
                move_forward()
    except:
        print "error!"
