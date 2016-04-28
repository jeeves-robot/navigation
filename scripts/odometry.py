import rospy
from geometry_msgs import PoseWithCovariance, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

odom_topic = 'robot_pose_ekf/odom_combined'
marker_publisher = rospy.Publisher('visualization_marker', Marker)

def show_text_in_rviz(self, text):
    marker = Marker(type=Marker.TEXT_VIEW_FACING, id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8), text=text)
    marker_publisher.publish(marker)

def distance(p1, p2):
    return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2) ** 0.5

def odom_callback(data):
    pose = data.pose.pose
    covariance = data.pose.covariance
    position = pose.position
    orientation = pose.orientation



if __name__ == '__main__':
    rospy.init_node('turtlebot_odometry')
    sub = rospy.Subscriber(odom_topic, PoseWithCovarianceStamped, odom_callback)
