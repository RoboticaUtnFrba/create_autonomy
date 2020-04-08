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
 * Desc: Cliff publisher plugin
 * Author: Steven Desvars
 * Date: 19 February 2020
 */

#include <ca_gazebo/create_cliff_plugin.h>

#include <string>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosCliff)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosCliff::GazeboRosCliff()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosCliff::~GazeboRosCliff()
{
  this->rosnode_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCliff::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  RayPlugin::Load(_parent, this->sdf);
  // Get the world name.
  std::string worldName = _parent->WorldName();
  this->world_ = physics::get_world(worldName);
  // save pointers
  this->sdf = _sdf;

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ =
      dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parent_ray_sensor_)
    gzthrow("GazeboRosCliff controller requires a Ray Sensor as its parent");

  this->robot_namespace_ = GetRobotNamespace(_parent, _sdf, "Laser");

  if (!this->sdf->HasElement("frameName"))
  {
    ROS_INFO_NAMED("laser", "Cliff plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");

  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO_NAMED("laser", "Cliff plugin missing <topicName>, defaults to /cliff");
    this->topic_name_ = "/cliff";
  }
  else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  this->laser_connect_count_ = 0;

  if (!this->sdf->HasElement("cliffValue"))
  {
    ROS_INFO_NAMED("laser", "Cliff plugin missing <cliffValue>, defaults to 5 cm");
    this->min_cliff_value = 0.05;
  }
  else
    this->min_cliff_value = _sdf->Get<double>("cliffValue");

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("laser",
        "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ROS_INFO_NAMED("laser", "Starting Cliff Plugin (ns = %s)", this->robot_namespace_.c_str());
  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
      boost::bind(&GazeboRosCliff::LoadThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCliff::LoadThread()
{
  this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gazebo_node_->Init(this->world_name_);

  this->pmq.startServiceThread();

  this->rosnode_.reset(new ros::NodeHandle(this->robot_namespace_));

  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);

  // resolve tf prefix
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao =
        ros::AdvertiseOptions::create<std_msgs::Bool>(
            this->topic_name_, 1,
            boost::bind(&GazeboRosCliff::LaserConnect, this),
            boost::bind(&GazeboRosCliff::LaserDisconnect, this),
            ros::VoidPtr(), NULL);
    this->pub_ = this->rosnode_->advertise(ao);
    this->pub_queue_ = this->pmq.addPub<std_msgs::Bool>();
  }

  // Initialize the controller

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosCliff::LaserConnect()
{
  this->laser_connect_count_++;
  if (this->laser_connect_count_ == 1)
    this->laser_scan_sub_ =
        this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                      &GazeboRosCliff::OnScan, this);
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosCliff::LaserDisconnect()
{
  this->laser_connect_count_--;
  if (this->laser_connect_count_ == 0)
    this->laser_scan_sub_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to ROS and publish if a cliff is detected or not
void GazeboRosCliff::OnScan(ConstLaserScanStampedPtr &_msg)
{
  std_msgs::Bool msg_output;
  msg_output.data = _msg->scan().ranges(0) > this->min_cliff_value;
  ROS_DEBUG_STREAM(this->parent_ray_sensor_->Name()
                    << " : [" << _msg->scan().ranges(0)
                    << " > " << this->min_cliff_value << "]");
  this->pub_queue_->push(msg_output, this->pub_);
}
}  // namespace gazebo
