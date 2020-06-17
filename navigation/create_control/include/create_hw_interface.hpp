#pragma once

#include "create/create.h"

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "create/create.h"

#include <ros/ros.h>

class Create2Interface : public hardware_interface::RobotHW {
public:
  Create2Interface(ros::NodeHandle &nh)
      : priv_nh_(nh), model_(create::RobotModel::CREATE_2) {

    hardware_interface::JointStateHandle state_handle_a(
        "wheel_left_joint", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b(
        "wheel_right_joint", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    registerInterface(&jnt_state_interface);

    // connect and register the joint velocity interface
    hardware_interface::JointHandle vel_handle_left(
        jnt_state_interface.getHandle("wheel_left_joint"), &cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_left);

    hardware_interface::JointHandle vel_handle_right(
        jnt_state_interface.getHandle("wheel_right_joint"), &cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_right);

    registerInterface(&jnt_vel_interface);
    std::string dev;
    int baud;
    priv_nh_.param<std::string>("dev", dev, "/dev/ttyUSB0");
    priv_nh_.param<int>("baud", baud, 115200);
    priv_nh_.param<std::string>("tf_prefix", tf_prefix_, "create1_tf");

    robot_.reset(new create::Create(model_));

    if (!robot_->connect(dev, baud)) {
      ROS_FATAL("[CREATE] Failed to establish serial connection with Create.");
      ros::shutdown();
    }
    ROS_INFO("[CREATE] Connection established");

    // Start in full control mode
    robot_->setMode(create::MODE_FULL);
  }

  void write() {
    robot_->driveWheels(cmd[0] * (model_.getWheelDiameter()),
                        cmd[1] * (model_.getWheelDiameter()));
  }

  void read() {
    pos[0] =
        robot_->getLeftWheelTotalDistance() / (model_.getWheelDiameter() / 2);
    pos[1] =
        robot_->getRightWheelTotalDistance() / (model_.getWheelDiameter() / 2);
  }

  ros::Time get_time() { return ros::Time::now(); }

  ros::Duration get_period() {
    auto ret = last_update_;
    last_update_ = ros::Time::now();
    return (ros::Time::now() - ret);
  }

private:
  const std::string str_base_footprint_ = "base_footprint";
  const std::string str_odom_ = "odom";
  std::string tf_prefix_;
  const std::string str_wheel_left_link_ = "wheel_left_link";
  const std::string str_wheel_right_link_ = "wheel_right_link";
  ros::NodeHandle priv_nh_;
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
  ros::Time last_update_;
  std::unique_ptr<create::Create> robot_;
  create::RobotModel model_;
};
