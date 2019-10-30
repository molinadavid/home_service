#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle n;

  // Publishers for the marker class
  ros::Publisher pickup_pub = n.advertise<std_msgs::String>("/add_markers/pickup",20);
  ros::Publisher dropoff_pub = n.advertise<std_msgs::String>("/add_markers/deliver",20);

  // Create an emtpy message to publish to the markers, this is just for simplicity sake
  std_msgs::String msg;
  msg.data = "";

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = -3.06897065347;
  goal.target_pose.pose.position.y = -4.0984235486;
  goal.target_pose.pose.orientation.z = 0.993520746783;
  goal.target_pose.pose.orientation.w = 0.113650894021;

  while (pickup_pub.getNumSubscribers() < 1 && dropoff_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Waiting for the markers subscribers subscribers");
    sleep(1);
  }

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal for pickup location");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the robot reached the counter, will pick up the food and deliver it.");
    pickup_pub.publish(msg);
    ros::Duration(5.0).sleep();
  }
  else
  {
    ROS_INFO("The robot failed to reach the counter, it will go to the bathroom to cry.");
    return 0;
  }

  goal.target_pose.pose.position.x = 9.26427726203;
  goal.target_pose.pose.position.y = -8.7255643016;
  goal.target_pose.pose.orientation.z = -0.276170431328;
  goal.target_pose.pose.orientation.w = 0.961108679006;

  ROS_INFO("Sending goal for drop off location");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("SUCCESS, the robot reached table, exiting customers will have their food, exiting");
    dropoff_pub.publish(msg);
    ros::Duration(5.0).sleep();
  }
  else
  {
    ROS_INFO("The robot failed to reach the table, the customers will not be happy, exiting");
  }
  return 0;
}