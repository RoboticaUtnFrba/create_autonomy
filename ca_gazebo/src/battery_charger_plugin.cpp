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
 * Desc: Model pose publisher
 * Author: Emiliano Borghi
 * Date: 11 January 2020
 */


#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sdf/sdf.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "ca_msgs/ChargingState.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"

#include <string>
#include <cmath>

static const ros::Duration update_period = ros::Duration(1);  // 10 ms

namespace gazebo
{
class BatteryChargerPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    GZ_ASSERT(_parent, "BatteryChargerPlugin _parent pointer is NULL");

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                       << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    ROS_INFO("Battery charger started!");
    // Store the pointer to the model
    this->model_ = _parent;
    ROS_INFO("model Name = %s", this->model_->GetName().c_str());
    this->indicator_link_ = this->model_->GetLink("link");
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                               std::bind(&BatteryChargerPlugin::OnUpdate, this));
    this->prev_update_time_ = ros::Time::now();

    this->rosnode_.reset(new ros::NodeHandle());

    this->dock_pose_sub_ = this->rosnode_->subscribe("/dock/pose", 1, &BatteryChargerPlugin::DockPoseCb, this);

    this->capacity_pub_ = this->rosnode_->advertise<std_msgs::Float32>("/battery/capacity", 1);
    this->charge_pub_ = this->rosnode_->advertise<std_msgs::Float32>("/battery/charge", 1);
    this->charge_ratio_pub_ = this->rosnode_->advertise<std_msgs::Float32>("/battery/charge_ratio", 1);
    this->current_pub_ = this->rosnode_->advertise<std_msgs::Float32>("/battery/current", 1);
    this->temperature_pub_ = this->rosnode_->advertise<std_msgs::Int16>("/battery/temperature", 1);
    this->voltage_pub_ = this->rosnode_->advertise<std_msgs::Float32>("/battery/voltage", 1);
    this->charging_state_pub_ = this->rosnode_->advertise<ca_msgs::ChargingState>("/battery/charging_state", 1);

    this->charge_.data = 5;
  }

  // Called by the world update start event
public:
  void OnUpdate()
  {
    if ((ros::Time::now() - this->prev_update_time_) < update_period)
    {
      return;
    }
    ignition::math::Vector3<double> robot_pose = this->model_->WorldPose().Pos();
    ignition::math::Vector3<double> robot_scale = this->model_->Scale();
    ROS_INFO("model Scale X=%f Y=%f Z%f ",
      robot_scale.X(), robot_scale.Y(), robot_scale.Z());
    ROS_INFO("model Pose X=%f Y=%f Z%f ",
      robot_pose.X(), robot_pose.Y(), robot_pose.Z());
    ROS_INFO("dock Pose X=%f Y=%f Z%f ",
      this->dock_pose_.x, this->dock_pose_.y, this->dock_pose_.z);
    float distance = sqrt(
      pow(this->dock_pose_.x - robot_pose.X(), 2) +
      pow(this->dock_pose_.y - robot_pose.Y(), 2)
    );
    ROS_INFO("Distance to dock = %f", distance);

    std_msgs::Float32 capacity;
    capacity.data = 6;
    this->capacity_pub_.publish(capacity);

    std_msgs::Float32 current;
    ca_msgs::ChargingState charging_state;

    if(distance<0.1) {
      //charging
      this->charge_.data += 0.05;
      current.data = 2;
      charging_state.state = 2; //Full charge
      if(this->charge_.data > capacity.data) {
        this->charge_.data = capacity.data;
        current.data = 0;
        charging_state.state = ca_msgs::ChargingState::CHARGE_TRICKLE; //Trickle charge
      }
    } else {
      //discharging
      this->charge_.data -= 0.05;
      current.data = -2;
      charging_state.state = 0; //none
      if(this->charge_.data < 0) {
        this->charge_.data = 0;
        current.data = 0;
      }
    }

    this->current_pub_.publish(current);
    this->charging_state_pub_.publish(charging_state);
    this->charge_pub_.publish(this->charge_);

    std_msgs::Float32 charge_ratio;
    charge_ratio.data = 1;
    this->charge_ratio_pub_.publish(charge_ratio);

    std_msgs::Int16 temperature;
    temperature.data = 40;
    this->capacity_pub_.publish(temperature);

    std_msgs::Float32 voltage;
    voltage.data = 12.1;
    this->capacity_pub_.publish(voltage);

    float scale = this->charge_.data/6;
    if(scale < 0.2) scale = 0.2;
    robot_scale.X(scale);
    robot_scale.Y(scale);
    robot_scale.Z(scale);
    this->model_->SetScale(robot_scale);
    // Update time
    this->prev_update_time_ = ros::Time::now();
  }

  void DockPoseCb(const geometry_msgs::Pose & msg)
  {
    ignition::math::Vector3<double> robot_pose = this->model_->WorldPose().Pos();
    this->dock_pose_ = msg.position;
  }

private:
  geometry_msgs::Point dock_pose_;
  std::shared_ptr<ros::NodeHandle> rosnode_;
  ros::Publisher capacity_pub_;
  ros::Publisher charge_pub_;
  ros::Publisher charge_ratio_pub_;
  ros::Publisher current_pub_;
  ros::Publisher temperature_pub_;
  ros::Publisher voltage_pub_;
  ros::Publisher charging_state_pub_;
  ros::Subscriber dock_pose_sub_;
  physics::ModelPtr model_;
  physics::LinkPtr indicator_link_;
  ros::Time prev_update_time_;
  std_msgs::Float32 charge_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(BatteryChargerPlugin)

}  // namespace gazebo
