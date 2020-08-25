/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, Emiliano Borghi
 *
 */
#ifndef CA_BEHAVIOR_TREE_CONDITIONS_IS_BATTERY_LEVEL_OK_H
#define CA_BEHAVIOR_TREE_CONDITIONS_IS_BATTERY_LEVEL_OK_H

#include <string>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>


class IsBatteryLevelOK : public BT::ConditionNode
{
private:
  double current_voltage_;
  double warning_voltage_;
  // The node that will be used for any ROS operations
  ros::Subscriber sub_;
public:
  IsBatteryLevelOK(const std::string& name, const BT::NodeConfiguration& config);
  ~IsBatteryLevelOK();
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

#endif  // CA_BEHAVIOR_TREE_CONDITIONS_IS_BATTERY_LEVEL_OK_H
