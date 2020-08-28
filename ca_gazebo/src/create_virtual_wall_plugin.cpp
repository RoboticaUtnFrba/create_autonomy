/*
 * Copyright 2020
 *     Emiliano Borghi
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: Virtual Wall plugin
 * Author: Emiliano Borghi
 * Date: 19 February 2020
 */

#include <cmath>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <ca_gazebo/create_virtual_wall_plugin.hh>

using gazebo::VirtualWallSensorPlugin;

static const char create2_model_name_prefix[] = "create";
static const size_t create2_model_name_prefix_length = sizeof(create2_model_name_prefix) - 1;
static const ros::Duration update_rate = ros::Duration(0.01);  // 100 Hz

GZ_REGISTER_SENSOR_PLUGIN(VirtualWallSensorPlugin)

void VirtualWallSensorPlugin::Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  this->sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(sensor);
  this->world_ = physics::get_world();
  this->sensor_->SetActive(true);
  ROS_DEBUG_NAMED("virtual_wall_plugin", "Loading");

  this->rcon_distance_ = 0.1;
  if (sdf->HasElement("rconDistance"))
    this->rcon_distance_ = sdf->Get<double>("rconDistance");

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "virtual_wall_node");
  }
  this->rosnode_ = std::make_unique<ros::NodeHandle>();
  this->updateNewEntity_ = event::Events::ConnectAddEntity(
                             std::bind(&VirtualWallSensorPlugin::OnAddEntity, this));
  this->OnAddEntity();
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
                              std::bind(&VirtualWallSensorPlugin::OnUpdate, this));
  ROS_DEBUG_NAMED("virtual_wall_plugin", "Loaded");
  this->prev_update_time_ = ros::Time::now();
}

void VirtualWallSensorPlugin::OnAddEntity()
{
  const auto & world_models = this->world_->Models();
  // Filter models which name starts with `create` and are continued with a number.
  create_2_models_.resize(0);
  pubs_.resize(0);
  std::copy_if(
    world_models.begin(),
    world_models.end(),
    std::back_inserter(this->create_2_models_),
    [this](const physics::ModelPtr & model)
  {
    const std::string & name = model->GetName();
    ROS_DEBUG_NAMED("virtual_wall_plugin", "checking model %s", name.c_str());
    if (name.size() < create2_model_name_prefix_length + 1)
    {
      ROS_DEBUG_NAMED("virtual_wall_plugin", "model name `%s` is short", name.c_str());
      return false;
    }
    if (std::strncmp(
          name.c_str(),
          create2_model_name_prefix,
          create2_model_name_prefix_length) != 0)
    {
      ROS_DEBUG_NAMED(
        "virtual_wall_plugin",
        "model name `%s` doesn't start with prefix `%s`",
        name.c_str(),
        create2_model_name_prefix);
      return false;
    }
    std::istringstream iss(name.substr(create2_model_name_prefix_length));
    size_t x;
    iss >> x;
    if (iss && iss.eof())
    {
      std::ostringstream oss;
      oss << "create" << x << "/virtual_wall/raw/";
      ROS_DEBUG_NAMED("virtual_wall_plugin", "model name `%s` ok", name.c_str());
      this->pubs_.emplace_back(
        this->rosnode_->advertise<std_msgs::Bool>(
          oss.str().c_str(), 1));
      return true;
    }
    ROS_DEBUG_NAMED("virtual_wall_plugin", "model name `%s` doesn't end with a number", name.c_str());
    return false;
  });  // NOLINT(whitespace/braces)
  ROS_DEBUG_NAMED("virtual_wall_plugin", "number of create2: %zu", create_2_models_.size());
}

void VirtualWallSensorPlugin::OnUpdate()
{
  if ((ros::Time::now() - this->prev_update_time_) < update_rate)
  {
    return;
  }
  double detected_range = this->sensor_->Range(0);
  ignition::math::Vector3d laser_dir(1., 0., 0.);
  ignition::math::Vector3d sensor_offset(0.145, 0., 0.068);
  const auto & parent_pose = world_->EntityByName(sensor_->ParentName())->WorldPose();
  const ignition::math::Pose3d & laser_pose = parent_pose + sensor_->Pose();

  laser_dir = laser_pose.Rot().RotateVector(laser_dir);
  ROS_DEBUG_NAMED("virtual_wall_plugin", "pose x=%f y=%f z=%f",
                  laser_pose.Pos().X(),
                  laser_pose.Pos().Y(),
                  laser_pose.Pos().Z());
  ROS_DEBUG_NAMED("virtual_wall_plugin", "dir x=%f y=%f z=%f",
                  laser_dir.X(),
                  laser_dir.Y(),
                  laser_dir.Z());

  for (size_t i = 0; i < create_2_models_.size(); i++)
  {
    const auto & model = this->create_2_models_[i];
    const auto & pub = this->pubs_[i];

    const ignition::math::Pose3d & robot_pose = model->WorldPose();
    sensor_offset = robot_pose.Rot().RotateVector(sensor_offset);
    ignition::math::Pose3d sensor_pose = robot_pose + sensor_pose;
    ROS_DEBUG_NAMED("virtual_wall_plugin", "sensor_pose x=%f y=%f z=%f",
                    sensor_pose.Pos().X(),
                    sensor_pose.Pos().Y(),
                    sensor_pose.Pos().Z());

    // https://www.qc.edu.hk/math/Advanced%20Level/Point_to_line.htm
    // Pythagoras Theorem
    // Get distance from the RCON (robot's omni sensor) to the VW sensor
    const auto & error = (sensor_pose.Pos() - laser_pose.Pos());
    // Get the projected error into the line generated by the VW sensor and its laser
    const double proj_error = error.Dot(laser_dir);
    // Get the distance from the robot to the line generated by the VW sensor and its laser
    // by doing a simple vector difference
    const double proj_error_sq = proj_error * proj_error;
    const double error_sq = error.Length() * error.Length();
    const double d = std::sqrt(error_sq - proj_error_sq);
    ROS_DEBUG_NAMED("virtual_wall_plugin", "d=%f detected_range=%f error=%f",
                    d,
                    detected_range,
                    error.Length());

    std_msgs::Bool msg;
    // The line constituted by VW laser has the from y = m*x + b
    // So to check if the robot is in the x < 0:
    // x = (y - b) / m, where 'y' is the component of the robot
    const double y = sensor_pose.Pos().Y();
    const double b = laser_pose.Pos().Y();
    const double m = laser_dir.Y() / laser_dir.X();
    const bool in_front_of_laser = (y - b) / m > 0;
    // If the robot detects the ray but it's interfering with another object
    const bool in_range = detected_range > (error.Length() - this->rcon_distance_) &&
                          error.Length() < this->sensor_->RangeMax();
    msg.data = d < 0.3 && in_range && in_front_of_laser;
    ROS_DEBUG_NAMED("virtual_wall_plugin", "result=%d",
                    msg.data);
    pub.publish(msg);
  }
  this->prev_update_time_ = ros::Time::now();
}
