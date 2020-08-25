/*
 * Copyright 2020
 *     Steven Desvars
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
 * Desc: Cliff message publisher plugin
 * Author: Steven Desvars
 * Date: 19 February 2020
 */

#include "ca_gazebo/cliff_msg_publisher.h"

#include <string>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboCliffMsgPublisher)

void GazeboCliffMsgPublisher::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "cliff_msg_publisher");
  }
  // Initialize the ros variables and gazebo variables
  if (!_sdf->HasElement("robotNamespace"))
  {
    ROS_INFO("Cliff Msg plugin: missing <robotNamespace>, defaults to create");
    this->robot_name_ = "create";
  }
  else this->robot_name_ = _sdf->Get<std::string>("robotNamespace");

  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("Cliff Msg plugin: missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else this->frame_name_ = _sdf->Get<std::string>("frameName");

  if (!_sdf->HasElement("updateRate"))
  {
    ROS_INFO("Cliff Msg plugin: missing <updateRate>, defaults to 10 Hz");
    this->update_period_ = 1 / 10.0;
  }
  else this->update_period_ = 1.0 / (_sdf->Get<double>("updateRate"));

  std::string topic_name = "";
  if (!_sdf->HasElement("topicName"))
  {
    ROS_INFO("Cliff Msg plugin: missing <topicName>, defaults to cliff");
    topic_name = "cliff";
  }
  else topic_name = _sdf->Get<std::string>("topicName");

  this->msg_.header.frame_id = this->frame_name_;

  this->side_left_sub_ = this->nh_.subscribe(
      this->robot_name_ + "/side_left_cliff_sensor/raw/", 1, &GazeboCliffMsgPublisher::SideLeftCliffCallback, this);
  this->side_right_sub_ = this->nh_.subscribe(
      this->robot_name_ + "/side_right_cliff_sensor/raw/", 1, &GazeboCliffMsgPublisher::SideRightCliffCallback, this);
  this->front_left_sub_ = this->nh_.subscribe(
      this->robot_name_ + "/front_left_cliff_sensor/raw/", 1, &GazeboCliffMsgPublisher::FrontLeftCliffCallback, this);
  this->front_right_sub_ = this->nh_.subscribe(
      this->robot_name_ + "/front_right_cliff_sensor/raw/", 1, &GazeboCliffMsgPublisher::FrontRightCliffCallback, this);

  this->publisher_ = this->nh_.advertise<ca_msgs::Cliff>(tf::resolve(this->robot_name_, topic_name), 1);
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboCliffMsgPublisher::OnUpdate, this));
  this->prev_update_time_ = ros::Time::now();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboCliffMsgPublisher::OnUpdate()
// Every update check if one of the sensors is active, if it is, change the output to true, publishes every 1ms
{
  if ((ros::Time::now() - this->prev_update_time_) < ros::Duration(this->update_period_))
  {
    return;
  }
  this->msg_.header.seq++;
  this->msg_.header.stamp = ros::Time::now();
  {
    const std::lock_guard<std::mutex> lock(side_left_mutex);
    this->msg_.is_cliff_left = this->get_side_left_;
  }
  {
    const std::lock_guard<std::mutex> lock(side_right_mutex);
    this->msg_.is_cliff_right = this->get_side_right_;
  }
  {
    const std::lock_guard<std::mutex> lock(front_left_mutex);
    this->msg_.is_cliff_front_left = this->get_front_left_;
  }
  {
    const std::lock_guard<std::mutex> lock(front_right_mutex);
    this->msg_.is_cliff_front_right = this->get_front_right_;
  }
  this->publisher_.publish(this->msg_);
  this->prev_update_time_ = ros::Time::now();
}

// Callback to get the data from the side left cliff sensor
void GazeboCliffMsgPublisher::SideLeftCliffCallback(const std_msgs::Bool::ConstPtr &data)
{
  const std::lock_guard<std::mutex> lock(side_left_mutex);
  this->get_side_left_ = data->data;
}

// Callback to get the data from the side right cliff sensor
void GazeboCliffMsgPublisher::SideRightCliffCallback(const std_msgs::Bool::ConstPtr &data)
{
  const std::lock_guard<std::mutex> lock(side_right_mutex);
  this->get_side_right_ = data->data;
}

// Callback to get the data from the front left cliff sensor
void GazeboCliffMsgPublisher::FrontLeftCliffCallback(const std_msgs::Bool::ConstPtr &data)
{
  const std::lock_guard<std::mutex> lock(front_left_mutex);
  this->get_front_left_ = data->data;
}

// Callback to get the data from the front right sensor
void GazeboCliffMsgPublisher::FrontRightCliffCallback(const std_msgs::Bool::ConstPtr &data)
{
  const std::lock_guard<std::mutex> lock(front_right_mutex);
  this->get_front_right_ = data->data;
}
}  // namespace gazebo
