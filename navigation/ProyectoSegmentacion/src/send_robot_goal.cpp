#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>
#include <sstream>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
const std::string NODE_NAME = "navigation_goal";

double inline deg2rad(double deg) { return deg * M_PI / 180.0; };

int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);
  MoveBaseClient ac("move_base", true);

  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server...");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  try
  {
    goal.target_pose.pose.position.x = std::stod(argv[1]);
    goal.target_pose.pose.position.y = std::stod(argv[2]);
    // Converting yaw angle in degrees to quaternion in radians
    const double yaw_deg = std::stod(argv[3]);
    tf2::Quaternion q;
    q.setRPY( 0, 0, deg2rad(yaw_deg) );
    goal.target_pose.pose.orientation = tf2::toMsg(q);
  }
  catch (int e)
  {
    ROS_WARN_STREAM_NAMED(NODE_NAME, "Using default 2D pose: [0 m, 0 m, 0 deg]");
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
  }
  ROS_INFO("Sending move base goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Robot has arrived to the goal position");
  }
  else
  {
    ROS_INFO("The base failed for some reason");
  }
  return 0;
}
