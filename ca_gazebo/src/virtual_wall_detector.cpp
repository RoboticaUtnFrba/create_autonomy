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
 * Desc: Virtual Wall detector
 * Author: Emiliano Borghi
 * Date: 19 February 2020
 */
#include "ca_gazebo/virtual_wall_detector.h"

#include <string>

static const char create2_model_name_prefix[] = "create";
static const size_t create2_model_name_prefix_length = sizeof(create2_model_name_prefix) - 1;

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboVirtualWallDetector)

void GazeboVirtualWallDetector::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->update_period_ = 1.0 / (_sdf->Get<double>("updateRate"));
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "virtual_wall_detector");
  }
  // Initialize the ros variables and gaebo variables
  const std::string & name = _parent->GetName();
  std::istringstream iss(name.substr(create2_model_name_prefix_length));
  size_t x;
  iss >> x;
  if (iss && iss.eof())
  {
    std::ostringstream oss;
    oss << "create" << x << "/virtual_wall/raw/";
    std::string topic_name = oss.str();
    this->subscriber_ = this->nh_.subscribe(topic_name, 1, &GazeboVirtualWallDetector::WallCallback, this);
    oss.str("");
    oss << "create" << x << "/virtual_wall/";
    topic_name = oss.str();
    this->publisher_ = this->nh_.advertise<std_msgs::Bool>(topic_name, 1);
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
                                std::bind(&GazeboVirtualWallDetector::OnUpdate, this));
    this->prev_update_time_ = ros::Time::now();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboVirtualWallDetector::OnUpdate()
// Every update check if one of the sensors is active, if it is, change the output to true, publishes every 1ms
{
  this->is_vwall_detected_ += this->get_vwall_;
  if ((ros::Time::now() - this->prev_update_time_) < ros::Duration(this->update_period_))
  {
    return;
  }
  std_msgs::Bool msg;
  msg.data = this->is_vwall_detected_;
  this->publisher_.publish(msg);
  this->is_vwall_detected_ = false;
  this->prev_update_time_ = ros::Time::now();
}

void GazeboVirtualWallDetector::WallCallback(const std_msgs::Bool::ConstPtr& data)
// Callback to get the data from the virtual wall sensors
{
  const std::lock_guard<std::mutex> lock(vwall_mutex);
  this->get_vwall_ = data->data;
}
}  // namespace gazebo
