#!/bin/sh
filepath=`pwd`
xterm  -e  " export TURTLEBOT_GAZEBO_WORLD_FILE=$filepath/../worlds/cafe.world && roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 3
xterm  -e  " export TURTLEBOT_GAZEBO_MAP_FILE=$filepath/../maps/cafe.yaml && roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 3
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e `rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  pose:
    position: {x: 2.93, y: -3.287, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.930, w: -0.367}
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"` &
sleep 3
xterm  -e  " rosrun pick_objects pick_objects "