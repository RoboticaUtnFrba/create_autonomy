#pragma once

#include <cmath>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <ros/ros.h>

namespace gazebo
{

class VirtualWallSensorPlugin : public SensorPlugin
{
public:
  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf);

  void OnUpdate();

  void OnAddEntity();

private:

  sensors::RaySensorPtr sensor_;
  physics::Model_V create_2_models_;
  physics::WorldPtr world_;
  event::ConnectionPtr updateConnection_;
  event::ConnectionPtr updateNewEntity_;
  ros::Time prev_update_time_;

  std::unique_ptr<ros::NodeHandle> rosnode_;
  std::vector<ros::Publisher> pubs_;
};

}
