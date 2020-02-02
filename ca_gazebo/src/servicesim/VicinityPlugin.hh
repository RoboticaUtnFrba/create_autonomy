/*
 * Copyright (C) 2018 Open Source Robotics Foundation, Inc.
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

#ifndef SERVICESIM_VICINITYPLUGIN_HH
#define SERVICESIM_VICINITYPLUGIN_HH

#include <string>
#include <vector>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>

#include <ca_msgs/ActorNames.h>
#include <ros/ros.h>

namespace servicesim
{
  /// \brief Reports the name of all actors within a given radius of the model
  ///
  /// SDF params:
  ///   * <threshold> Radius in meters, defaults to 5.0
  ///   * <topicName> ROS topic name: /<robotName>/<topicName>, defaults to RFID
  ///   * <updateRate> Publish frequency in Hz
  class VicinityPlugin: public gazebo::ModelPlugin
  {
    /// \brief Constructor
    public: VicinityPlugin();

    /// \brief Destructor
    public: virtual ~VicinityPlugin();

    // Documentation inherited
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Called at every simulation iteration
    public: void Update();

    /// \brief Store pointer to the model
    private: gazebo::physics::ModelPtr model_;

    /// \brief Store pointer to the world
    private: gazebo::physics::WorldPtr world_;

    /// \brief Update event connection
    private: gazebo::event::ConnectionPtr updateConnection;

    /// \brief Radius in meters
    private: double threshold_;

    /// \brief Publish frequency
    private: double update_rate_;

    /// \brief Topic name
    private: std::string topicName_;

    /// \brief Last update time
    private: gazebo::common::Time last_time_;

    /// \brief Store pointers to all actors in the world
    private: std::vector<gazebo::physics::ActorPtr> actorPtrs_;

    /// \brief ROS node handle
    private: ros::NodeHandle *rosnode_;

    /// \brief ROS publisher
    private: ros::Publisher vicinity_pub_;
  };
}

#endif
