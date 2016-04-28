

odom_topic = 'robot_pose_ekf/odom_combind'
marker_publisher = rospy.Publisher('visualization_marker', Marker)

def distance(p1, p2): 
        return ((p1.x - p2.x)**2 + (p1.y - p2.y)** 2 + (p1.z - p2.z)**2) ** 0.5

def odom_callback(data): 
    pose = data.pose.pose
    covariance = data.pose.covariance
    position = pose.position
    orientation = pose.orientation


if __name__ == '__main__':
    rospy.init_node('turtlebot_odometry')
    sub = rospy.Subscriber(odom_topic, PoseWithCovarianceStamped, odom_callback)

