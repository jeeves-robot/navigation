#!/usr/bin/env python

import rospy
import tf
import turtlesim.srv
import time
import math
import sys
import csv

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, Twist
from std_msgs.msg import Header, ColorRGBA

base_position = 'map'
displaced_position = 'base_link'

class Annotator:
	def __init__(self, filename):
		self._filename = filename
		self._num_markers = 0
		rospy.init_node('turtlebot_annotater')
		# Create a TF listener
		self._listener = tf.TransformListener()
		self._rate = rospy.Rate(10.0)
		self._marker_publisher = rospy.Publisher('visualization_marker', Marker)

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
		self.write_data(name, pointPosition, quaternion, self._num_markers)

	def write_data(self, name, point_position, quaternion, marker_id):
		with open(self._filename, 'a') as csvfile:
			writer = csv.writer(csvfile, delimiter='\t', quotechar='|', quoting=csv.QUOTE_MINIMAL)
			writer.writerow([name, point_position, quaternion, marker_id])

	def go_to_marker(self, name):
		# TODO
		pass


if __name__ == '__main__':
	filename = sys.argv[1]
	annotate = Annotator(filename)
	while(1):
		terms = raw_input('Save or go to marker? s to save, g to go to marker')
		args = terms.split(" ")
		if args[0] == 's':
			annotate.place_marker(args[1])
		else:
			annotate.go_to_marker(args[1])
			
		
