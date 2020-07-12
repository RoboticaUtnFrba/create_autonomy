#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include "actionlib_msgs/GoalID.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include <math.h>
#include <iostream>
#include <sstream>
#include <string>

#define STATE_AWAIT_GOAL 0
#define STATE_PUBLISH_GOAL 1
#define STATE_GET_RESULT 2
#define STATE_PUBLISH_RESULT 3
#define STATE_CANCEL 4

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;
const std::string NODE_NAME = "navigation_goal";

int node_state = STATE_AWAIT_GOAL;
geometry_msgs::Vector3 goal;
std_msgs::Float32 distance;
std_msgs::String result;
actionlib_msgs::GoalID cancel_msg;

// Called when a new goal is received.
void onNewGoalCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
  if (node_state != STATE_AWAIT_GOAL) {
    return;
  }
  ROS_INFO("New goal received! (x = %.2f, y = %.2f)", msg->x, msg->y);
  goal.x = msg->x;
  goal.y = msg->y;
  goal.z = msg->z;
  node_state = STATE_PUBLISH_GOAL;
}

// Called when a goal is canceled.
void onCancelGoalCallback(const std_msgs::Float32::ConstPtr& msg) {
  if (msg->data == 0 && node_state == STATE_GET_RESULT) {
    node_state = STATE_CANCEL;
  }
}

// Called when move base has a result.
void onResultCallback(
    const move_base_msgs::MoveBaseActionResult::ConstPtr& msg) {
  switch (msg->status.status) {
    case 3: {
      result.data = "Robot has arrived to the goal position";
      ROS_INFO("Robot has arrived to the goal position");
      break;
    }
    case 2: {
      result.data = "The goal has been canceled.";
      ROS_INFO("The goal has been canceled.");
      break;
    }
    default: {
      result.data = "The base failed for some reason";
      ROS_INFO("The base failed for some reason");
    }
  }
  ROS_INFO("Waiting for a new goal...");
  node_state = STATE_PUBLISH_RESULT;
}

// Called on every feedback message from move base.
void onFeedbackCallback(
    const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
  float deltaX = goal.x - msg->feedback.base_position.pose.position.x;
  float deltaY = goal.y - msg->feedback.base_position.pose.position.y;
  distance.data = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
}

int main(int argc, char** argv) {
  std::string base_topic = argv[1];
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle n;
  MoveBaseClient ac("move_base", true);

  // Subscribes to goal messages.
  ros::Subscriber sub_goal = n.subscribe(base_topic, 1000, onNewGoalCallback);

  // Subscribes to move base result messages.
  ros::Subscriber sub_result =
      n.subscribe("move_base/result", 1000, onResultCallback);

  // Subscribes to move base feedback messages.
  ros::Subscriber sub_feedback =
      n.subscribe("move_base/feedback", 1000, onFeedbackCallback);

  // Subscribes to cancel-goal messages.
  ros::Subscriber sub_cancel =
      n.subscribe(base_topic + "_cancel", 1000, onCancelGoalCallback);

  // Publishes the remaining distance to the goal in meters.
  ros::Publisher pub_distance =
      n.advertise<std_msgs::Float32>(base_topic + "_distance", 1000);

  // Publishes the action restult.
  ros::Publisher pub_result =
      n.advertise<std_msgs::String>(base_topic + "_result", 1000);

  // Publishes cancel-goal messages.
  ros::Publisher pub_cancel =
      n.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1000);

  // Wait for move base action server.
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server...");
  }

  // Frecuency of loop in Hz
  ros::Rate loop_rate(10);
  ROS_INFO("Waiting for a new goal...");

  while (ros::ok()) {
    switch (node_state) {
      // Publishes the goal to the move base server.
      case STATE_PUBLISH_GOAL: {
        move_base_msgs::MoveBaseGoal move_base_goal;
        move_base_goal.target_pose.header.frame_id = "map";
        move_base_goal.target_pose.header.stamp = ros::Time::now();
        try {
          move_base_goal.target_pose.pose.position.x = goal.x;
          move_base_goal.target_pose.pose.position.y = goal.y;
        } catch (int e) {
          ROS_WARN_STREAM_NAMED(NODE_NAME,
                                "Using default 2D pose: [0 m, 0 m, 0 deg]");
          move_base_goal.target_pose.pose.position.x = 0.0;
          move_base_goal.target_pose.pose.position.y = 0.0;
        }
        move_base_goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("Sending move base goal");
        ac.sendGoal(move_base_goal);
        node_state = STATE_GET_RESULT;
        break;
      }
      // Publishes the goal result.
      case STATE_PUBLISH_RESULT: {
        pub_result.publish(result);
        node_state = STATE_AWAIT_GOAL;
        break;
      }
      // Publishes a cancel message to move base.
      case STATE_CANCEL: {
        pub_cancel.publish(cancel_msg);
        node_state = STATE_AWAIT_GOAL;
        break;
      }
    }

    // Publish the distance to the goal.
    pub_distance.publish(distance);

    // For callbacks
    ros::spinOnce();
    // Go to sleep for the configured interval
    loop_rate.sleep();
  }
  return 0;
}