/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef CA_GAZEBO_CREATE_IMU_PLUGIN_H
#define CA_GAZEBO_CREATE_IMU_PLUGIN_H

#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
class GazeboRosCreateIMU : public ModelPlugin
{
public:
  /// \brief Constructor
  GazeboRosCreateIMU();

  /// \brief Destructor
  virtual ~GazeboRosCreateIMU();

  /// \brief Load the controller
  /// \param node XML config node
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  /// \brief Update the controller
  virtual void UpdateChild();
private:
  /// \brief The parent World
  physics::WorldPtr world_;

  /// \brief The parent Model
  physics::ModelPtr model_;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

  /// \brief pointer to gazebo ros node
  GazeboRosPtr gazebo_ros_;
  ros::Publisher pub_;
  PubQueue<sensor_msgs::Imu>::Ptr pub_Queue;

  /// \brief ros message
  sensor_msgs::Imu imu_msg_;

  /// \brief store link name
  std::string link_name_;

  /// \brief store frame name
  std::string frame_name_;

  /// \brief topic name
  std::string topic_name_;

  /// \brief allow specifying constant xyz and rpy offsets
  ignition::math::Pose3d offset_;

  /// \brief A mutex to lock access to fields
  /// that are used in message callbacks
  boost::mutex lock_;

  /// \brief save last_time
  common::Time last_time_;
  ignition::math::Vector3d last_vpos_;
  ignition::math::Vector3d last_veul_;
  ignition::math::Vector3d apos_;
  ignition::math::Vector3d aeul_;

  // rate control
  double update_rate_;

  /// \brief: keep initial pose to offset orientation in imu message
  ignition::math::Pose3d initial_pose_;

  /// \brief Gaussian noise
  double gaussian_noise_;

  /// \brief Gaussian noise generator
  double GaussianKernel(double mu, double sigma);

  /// \brief call back when using service
  bool ServiceCallback(const std_srvs::Empty::Request &req,
                        const std_srvs::Empty::Response &res);

  ros::ServiceServer srv_;
  std::string service_name_;

  ros::CallbackQueue imu_queue_;
  void IMUQueueThread();
  boost::thread callback_queue_thread_;

  // Pointer to the update event connection
  event::ConnectionPtr update_connection_;

  // deferred load in case ros is blocking
  sdf::ElementPtr sdf_;
  void LoadThread();
  boost::thread deferred_load_thread_;
  unsigned int seed;

  // ros publish multi queue, prevents publish() blocking
  PubMultiQueue pmq;
};
}  // namespace gazebo
#endif  // CA_GAZEBO_CREATE_IMU_PLUGIN_H
