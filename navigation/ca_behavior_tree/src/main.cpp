// BehaviorTree.CPP
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

// ROS
#include <ros/ros.h>

#include "ca_behavior_tree/actions/movebase_client.h"
#include "ca_behavior_tree/actions/check_low_battery.h"


int main(int argc, char **argv) {

  ros::init(argc, argv, "test_bt");

  ros::NodeHandle nh("~");
  std::string xml_filename;
  if (!nh.getParam("xml_tree", xml_filename)) {
    ROS_FATAL_STREAM("XML behavior tree not found : "<< xml_filename);
  }
  ROS_INFO_STREAM("Loading XML : " << xml_filename);

  // We use the BehaviorTreeFactory to register our custom nodes
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<MoveBase>("MoveBase");
  factory.registerNodeType<BatteryCharging>("BatteryCharging");
  // factory.registerSimpleCondition("CheckBattery", CheckBattery, {BT::InputPort<int>("wait_tick")});

  // Trees are created at deployment-time (i.e. at run-time, but only once at
  // the beginning). The currently supported format is XML. IMPORTANT: when the
  // object "tree" goes out of scope, all the TreeNodes are destroyed
  auto tree = factory.createTreeFromFile(xml_filename);

  // Create a logger
  BT::StdCoutLogger logger_cout(tree);

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  // Keep on ticking until you get either a SUCCESS or FAILURE state
  while (ros::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    // Sleep 100 milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
