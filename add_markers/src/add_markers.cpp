#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"


class add_markers
{
private:
  void update_marker();
  void add_pickup();
  void pickup_cb(const std_msgs::String::ConstPtr& msg);
  void dropoff_cb(const std_msgs::String::ConstPtr& msg);
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Subscriber pickup_sub;
  ros::Subscriber deliver_sub;
  visualization_msgs::Marker marker;
public:
  add_markers();
};

add_markers::add_markers()
{
  ROS_WARN("Starting the add_markers node");
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  pickup_sub = n.subscribe("/add_markers/pickup",1,&add_markers::pickup_cb, this);
  deliver_sub = n.subscribe("/add_markers/deliver",1,&add_markers::dropoff_cb, this);

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_markers";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  
  add_pickup();
}

void add_markers::add_pickup()
{
  ROS_WARN("Publishing pickup marker");
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = -3.06897065347;
  marker.pose.position.y = -4.0984235486;

  update_marker();
}

void add_markers::pickup_cb(const std_msgs::String::ConstPtr& msg)
{
  ROS_WARN("Removing pickup marker");
  marker.action = visualization_msgs::Marker::DELETE;
  update_marker();
}

void add_markers::dropoff_cb(const std_msgs::String::ConstPtr& msg)
{
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 9.26427726203;
  marker.pose.position.y = -8.7255643016;

  update_marker();
}

void add_markers::update_marker()
{
  marker_pub.publish(marker);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  add_markers add_markers;
  ros::Rate r(1);
  ros::spin();

  return 0;
}
