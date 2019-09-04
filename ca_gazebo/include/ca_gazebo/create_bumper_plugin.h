#pragma once

/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
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
 * Desc: Bumper Controller
 * Author: Nate Koenig mod by John Hsu
 * Date: 24 Sept 2008
 */
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <std_msgs/String.h>
#include <ca_msgs/Bumper.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/tf.h>

namespace gazebo
{
  /// \brief A Bumper controller
  class CreateBumperPlugin : public SensorPlugin
  {
    /// Constructor
    public: CreateBumperPlugin();

    /// Destructor
    public: virtual ~CreateBumperPlugin();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// Update the controller
    protected: void OnUpdate();

  private:

      inline float normalizeAngle(float angle) {
        angle = std::fmod(angle + M_PI,2*M_PI);
        if (angle < 0)
            angle += 2*M_PI;
        return angle - M_PI;
      };

      /// \brief pointer to ros node
      std::shared_ptr<ros::NodeHandle> rosnode_;
      ros::Publisher contact_pub_;
      ros::Subscriber gts_sub_;
      ca_msgs::Bumper bumper_event_;
      void PublishBumperMsg();

      void GtsCb(const nav_msgs::Odometry::ConstPtr& msg);
      double robot_heading_;

      bool bumper_left_was_pressed_;
      bool bumper_center_was_pressed_;
      bool bumper_right_was_pressed_;
      bool bumper_left_is_pressed_;
      bool bumper_center_is_pressed_;
      bool bumper_right_is_pressed_;
      
      /// \brief set topic name of broadcast
      std::string bumper_topic_name_;
      std::string frame_name_;
      
      sensors::ContactSensorPtr bumper_;
      
      /// \brief for setting ROS namespace
      std::string robot_namespace_;
      
      // Pointer to the update event connection
      event::ConnectionPtr update_connection_;
  };

}
