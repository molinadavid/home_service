# Home service robot project
For this project the final idea is to have a Turtlebot 2 running in simulation, run the AMCL package after creating a map of the environment using Gmapping to autonomously pick up a virtual object and deliver it to a different location.

For this project I decided to use a predefined Gazebo world (Big cafe) that I used for making the RTABMap project, because of the wide area of the world I decided to add a Hokuyo Range finder sensor to the standard stack to assist with the mapping and localization process.

## Main ROS packages
The main packages that I used for this project are the following.


### Gazebo
For the simulation of the robot I used [Gazebo](http://gazebosim.org/tutorials?tut=ros_overview)

### Tutrlebot 2
The robot used for this project is the Turtlebot 2 for that I used the [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo) package in order to take care of the simulation part.
I used the [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop) package in order to control the robot manually while making the map.
In order to use pre made Rviz configurations for the Turtlebot 2 I used the [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers).

### Gmapping
The Ros [Gmapping](http://wiki.ros.org/gmapping) package is a wrapper for [Open slam Gmapping](http://wiki.ros.org/openslam_gmapping?distro=melodic) that is a laser-based SLAM to create a 2D map occupancy grid map.

## Custom ROS packages
The packages that I created for this project are the following.

### pick_objects
Is a small package to simulate the Pick and delivery of objects using the robot. This package has 2  pre-defined goals (Pose) on will simulate the pickup place where the robot will go to pick up the virtual item, after waiting for 5 seconds the robot will then go to the next location to deliver the virtual item.

### add_markers
Is the package that will add the virtual items to be visualized on the Rviz screen, when the package is first launched it will create a virtual item for the robot to pickup, after the robot arrives to the pickup location the item will be removed from the view and it will re appear on the drop off location once the robot arrives to it.

## Communication
The communication between the 2 packages was done using topics in order to keep it simple, When the robot reaches the pickup destination it will publish the topic so the add_markers package can remove the virtual item from view, once the robot reaches the drop off position it will again inform it so the package can place the virtual item again.

Another way this could have been achieved would be to subscribe the add_markers package to the `/move_base/result`, the `/move_base_simple/goal` and `/amcl_pose`, when the first goal is published it can then assume that is the pick up location (this is not always true and is one of the reasons I decided to avoid it) it can then place the virtual item in the give Pose and wait for the result of the move and if the robot is close from the item it can then remove it and assume that the next goal published will be the drop off location, it can then repeat the same procedure and wait for the robot to reach the destination and place the item there if is close from it.

## Scripts
All the scripts are under the `scripts` directory I tried to keep all the required path relative to that directory in order to automatically load the needed files to use on this project, the main script that will start the full project is `home_service.sh`