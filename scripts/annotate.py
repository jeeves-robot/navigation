#!/usr/bin/env python

import rospy
import tf
import turtlesim.srv
import time
import math
import sys
import csv
import actionlib

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, Twist
from std_msgs.msg import Header, ColorRGBA
from move_base_msgs.msg import *

base_position = 'map'
displaced_position = 'base_link'
fieldnames=['name', 'posX', 'posY', 'posZ', 'quat0', 'quat1', 'quat2', 'quat3', 'markerNum']

class Annotator:
	def __init__(self, filename):
		self._markers = {}
		self._filename = filename
		self._num_markers = 0
                self._goal_id=0

                # Load in any previously saved markers
                with open(self._filename, 'r') as csvfile:
                    reader = csv.DictReader(csvfile,
                            fieldnames=fieldnames, delimiter='\t', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                    for row in reader:
                        self._markers[row['name']]=row

                # Set up node for annotater
                rospy.init_node('turtlebot_annotater')
		# Create a TF listener
		self._listener = tf.TransformListener()
		self._rate = rospy.Rate(10.0)
		self._marker_publisher = rospy.Publisher('visualization_marker', Marker)
                # Set up action client to send goals to
                self._action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                self._action_client.wait_for_server()

	def place_marker(self, name):
                now = rospy.Time.now()
                # This is made up of a position and a quarternion
                self._listener.waitForTransform(base_position, displaced_position, now, rospy.Duration(20.0))
                (pos, quat) = self._listener.lookupTransform(base_position, displaced_position, rospy.Time(0))

		pointPosition = Point(pos[0], pos[1], pos[2])
		quaternion = Quaternion(quat[0], quat[1], quat[2], quat[3])
		self._num_markers += 1
		print 'putting down a marker'
		marker = Marker(type=Marker.SPHERE, id=self._num_markers,
			    lifetime=rospy.Duration(),
			    pose=Pose(pointPosition, quaternion),
			    scale=Vector3(0.06, 0.06, 0.06),
			    header=Header(frame_id=base_position),
			    color=ColorRGBA(0.0, 1.0, 0.0, 0.8), text=name)
		self._marker_publisher.publish(marker)

                newMarker = {}
                newMarker['name'] = name
                newMarker['posX'] = pos[0]
                newMarker['posY'] = pos[1]
                newMarker['posZ'] = pos[2]
                newMarker['quat0'] = quat[0]
                newMarker['quat1'] = quat[1]
                newMarker['quat2'] = quat[2]
                newMarker['quat3'] = quat[3]
                newMarker['markerNum'] = self._num_markers
                self._markers[name] = newMarker
                self.write_data(newMarker)

	def write_data(self, marker):
		with open(self._filename, 'a') as csvfile:
			writer = csv.DictWriter(csvfile,
                                fieldnames=marker.keys(),
                                delimiter='\t', quotechar='|', quoting=csv.QUOTE_MINIMAL)
			writer.writerow(marker)

	def go_to_marker(self, name):
                self._goal_id += 1
                marker = self._markers[name]
                point_position = Point(marker['posX'], marker['posY'], marker['posZ'])
                quaternion = Quaternion(marker['quat0'], marker['quat1'], marker['quat2'], marker['quat3'])
		print(point_position)
		print(quaternion)

                # TODO: Set up goal
                specificGoal = MoveBaseGoal()
                specificGoal.target_pose.header.frame_id = 'map'
                #specificGoal.target_pose.header.stamp = rospy.Time.now()

                pose = Pose(point_position, quaternion)
                specificGoal.target_pose.pose = pose

                self._action_client.send_goal(specificGoal)
                finished_on_time = self._action_client.wait_for_result(Duration.from_sec(30))
                if not finished_on_time:
                    self._action_client.cancel_goal()
                    print "Timed out acheiving goal"
                else:
                    if self._action_client.get_state() == GoalStatus.SUCCEEDED:
                        print "Success!"
                    else:
                        print "Failure"

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "Pass file location as argument to command line."
        print "Defaulting to markers.cv"
        filename = 'markers.cv'
    else:
	filename = sys.argv[1]
    annotate = Annotator(filename)
    while(1):
            terms = raw_input('Save or go to marker? (s name) save, (g name) go to marker: ')
            args = terms.split(" ")
            if args[0] == 's':
                    annotate.place_marker(args[1])
            elif args[0] == 'g':
                    annotate.go_to_marker(args[1])
