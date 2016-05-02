# Overview of commands to run to get the robot to move 

## Start gazebo
`roslaunch turtlebot_gazebo turtlebot_world.launch`

## Start rviz
`roslaunch rviz rviz`

1. Add pointcloud2
    1. Change topic: /camera/depth/points
1. Add Marker 
    1. Change topic: visualization_marker
1. Add TF (sometimes)
1. Fixed Frame: base_footprint or base_link


## Joystick 
`roslaunch kobuki_keyop keyop.launch`

## Start odom tracker
`rosrun robot_pose_ekf robot_pose_ekf

### Example code for sending goals to robot
http://ros-by-example.googlecode.com/svn/trunk/rbx_vol_1/rbx1_nav/nodes/move_base_square.py
