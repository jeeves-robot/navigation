#!/usr/bin/env python

import rospy
import time
import math

from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

from tf.transformations import euler_from_quaternion

odom_topic = 'robot_pose_ekf/odom_combined'
vel_topic = '/cmd_vel_mux/input/navi'

base_position = 'odom'
displaced_position = 'base_link'

class Homer:
    def __init__(self):
        rospy.init_node('turtlebot_homer')
        self._odom_subscriber = rospy.Subscriber(odom_topic,
                PoseWithCovarianceStamped, self.odom_callback)
        self._twist_publisher = rospy.Publisher(vel_topic, Twist, queue_size=5)
        self._active = False

    def distance(self, p1, p2):
        return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2) ** 0.5

    def odom_callback(self, stampedPose):
        if(self._active):
            pose = stampedPose.pose.pose
            position = pose.position
            if(self.distance(position, Point(0,0,0)) > .1):
                # Publish Twist that will get us closer to position
                twist = Twist()

                # First, figure out what angle we want to turn to
                quaternion = (pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w)
                euler_angle = euler_from_quaternion(quaternion)
                z_angle = euler_angle[2]

                # If we are not at the right angle, adjust angle
                if(abs(z_angle) > 0.05):
                    # Set the angular velocity so that it will reach this angle in .2 seconds
                    twist.angular.z = z_angle * -5
                    # Set a maximum, so we don't turn too fast
                    twist.angular.z = max(min(twist.angular.z, abs(z_angle)), -abs(z_angle))

                    print "Turning ", twist.angular.z
                    print "Angle is ", euler_angle[2] 
                    # Move in that direction for .2 seconds, then stop moving
                    self._twist_publisher.publish(twist)
                    time.sleep(.2)
                    self._twist_publisher.publish(Twist())
                # Otherwise, move towards where we want to go for .2 seconds, then stop
                else:
                    # Assumes x is always positive if angle is this small
                    # Can we assume that
                    print "Displacement is ", position.x
                    twist.linear.x = max(min(1, position.x * -5), -1)
                    print "Moving ", twist.linear.x
                    self._twist_publisher.publish(twist)
                    time.sleep(.2)
                    self._twist_publisher.publish(Twist())

    def activate(self):
        self._active = True

    def deactivate(self):
        self._active = False

if __name__ == '__main__':
    homer = Homer()
    homer.activate()

    rospy.spin()
