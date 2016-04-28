import rospy
import tf
import turtlesim.srv

import math

from geometry_msgs import PoseWithCovariance, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, Twist
from std_msgs.msg import Header, ColorRGBA

#odom_topic = 'robot_pose_ekf/odom_combined'

base_position = 'odom_combined'
displaced_position = 'base_link'

class Tracker:
    def __init__(self):
        rospy.init_node('turtlebot_odometry')

        # Create a TF listener
        self._listener = tf.TransformListener()
        self._rate = rospy.Rate(10.0)

        self._marker_publisher = rospy.Publisher('visualization_marker', Marker)
        self._last_position = Point()

    def show_text_in_rviz(self, text):
        # We show this relative to base_link
        # If we place the marker relative to the displaced_position,
        #   it will be near where the robot currently is.
        # That's why the pose is very close to 0, instead of a displacement
        marker = Marker(type=Marker.LINE_STRIP, id=0,
                    lifetime=rospy.Duration(1.5),
                    pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0,0,0,1)),
                    scale=Vector3(0.06, 0.06, 0.06),
                    header=Header(frame_id=displaced_position),
                    color=ColorRGBA(0.0, 1.0, 0.0, 0.8), text=text)
        self._marker_publisher.publish(marker)

    def distance(self, p1, p2):
        return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2) ** 0.5

    def odom_callback(self, pose):
        position = pose.position

        if(self.distance(position, self._last_position) > 1):
            self._last_position = position
            self.show_text_in_rviz("marker")

    def run(self):
        self._listener.waitForTransform(base_position, displaced_position, rospy.Time(), rospy.Duration(4.0))
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                # This is made up of a position and a quarternion
                self._listener.waitForTransform(base_position, displaced_position, now, rospy.Duration(4.0))
                transform = self._listener.lookupTransform(base_position, displaced_position, rospy.Time(0))
                self.odom_callback(transform)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            # Math to get the angular and linear from the transform listener
            # angular = 4 * math.atan2(trans[1], trans[0])
            # linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)

            self._rate.sleep()


if __name__ == '__main__':
    tracker = Tracker()
    tracker.run()