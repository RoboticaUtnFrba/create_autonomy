/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, Emiliano Borghi
 *
 */
#include <string>

#include <ca_behavior_tree/conditions/is_battery_level_ok.h>


IsBatteryLevelOK::IsBatteryLevelOK(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode::ConditionNode(name, config)
  // Set a high initial value
  , current_voltage_(14.4)
{
  std::string topic;
  if (!getInput<std::string>("topic", topic))
  {
    throw BT::RuntimeError("Missing required input [topic]");
  }

  // Battery plugin stops publishing at ~0.105
  if (!getInput<double>("warning_voltage", warning_voltage_))
  {
    throw BT::RuntimeError("Missing required input [warning_voltage]");
  }

  ros::NodeHandle nh_;
  // Battery voltage subscription
  sub_ = nh_.subscribe<std_msgs::Float32>(topic, 1,
      [this](const std_msgs::Float32::ConstPtr& msg)
      {
        current_voltage_ = msg->data;
      });  // NOLINT(whitespace/braces)
}

IsBatteryLevelOK::~IsBatteryLevelOK()
{
};

BT::NodeStatus IsBatteryLevelOK::tick()
{
  if (current_voltage_ <= warning_voltage_)
  {
    ROS_WARN_STREAM("Battery voltage is less than " << warning_voltage_ << " V");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList IsBatteryLevelOK::providedPorts()
{
  return
  {
    BT::InputPort<std::string>("topic"),
    BT::InputPort<double>("warning_voltage"),
  };
}
