/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, Emiliano Borghi
 *
 */
#ifndef CA_BEHAVIOR_TREE_ACTIONS_MOVEBASE_CLIENT_H
#define CA_BEHAVIOR_TREE_ACTIONS_MOVEBASE_CLIENT_H

#include <string>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <behaviortree_cpp_v3/action_node.h>


// Custom type
struct Pose2D
{
  double x, y, theta;
};


namespace BT
{
template <> inline
Pose2D convertFromString(StringView key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3)
  {
    throw BT::RuntimeError("invalid input)");
  }
  else
  {
    Pose2D output;
    output.x     = convertFromString<double>(parts[0]);
    output.y     = convertFromString<double>(parts[1]);
    output.theta = convertFromString<double>(parts[2]);
    return output;
  }
}
}  // end namespace BT

//----------------------------------------------------------------

class MoveBase : public BT::AsyncActionNode
{
public:
  MoveBase(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config)
  {
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    return
    {
      BT::InputPort<Pose2D>("goal"),
      BT::InputPort<std::string>("robot"),
    };
  }

  BT::NodeStatus tick() override;

  void halt() override
  {
    _aborted.store(true);
  }

private:
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  std::atomic_bool _aborted;
};

#endif  // CA_BEHAVIOR_TREE_ACTIONS_MOVEBASE_CLIENT_H
