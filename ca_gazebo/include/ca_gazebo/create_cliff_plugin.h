/*
 * Copyright 2020 Steven Desvars
 */
#ifndef CA_GAZEBO_CREATE_CLIFF_PLUGIN_H
#define CA_GAZEBO_CREATE_CLIFF_PLUGIN_H

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <string>
#include <vector>

#include <gazebo/transport/transport.hh>
#include <gazebo/plugins/RayPlugin.hh>

#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo_plugins/PubQueue.h>
#include <std_msgs/Bool.h>

namespace gazebo
{
class GazeboRosCliff : public RayPlugin
{
public:
  /// \brief Constructor
  GazeboRosCliff();

  /// \brief Destructor
  ~GazeboRosCliff();

  /// \brief Load the plugin
  /// \param take in SDF root element
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

private:
  /// \brief Keep track of number of connctions
  int laser_connect_count_;
  void LaserConnect();
  void LaserDisconnect();

  // Pointer to the model
  std::string world_name_;
  physics::WorldPtr world_;
  /// \brief The parent sensor
  sensors::RaySensorPtr parent_ray_sensor_;

  /// \brief pointer to ros node
  std::shared_ptr<ros::NodeHandle> rosnode_;
  ros::Publisher pub_;
  PubQueue<std_msgs::Bool>::Ptr pub_queue_;

  /// \brief topic name
  std::string topic_name_;

  /// \brief frame transform name, should match link name
  std::string frame_name_;

  /// \brief for setting ROS name space
  std::string robot_namespace_;

  // deferred load in case ros is blocking
  sdf::ElementPtr sdf;
  void LoadThread();
  boost::thread deferred_load_thread_;
  unsigned int seed;

  gazebo::transport::NodePtr gazebo_node_;
  gazebo::transport::SubscriberPtr laser_scan_sub_;
  void OnScan(const ConstLaserScanStampedPtr &_msg);

  /// Auxiliar variables to get the info from the Ray Sensor
  float min_cliff_value;

  /// \brief prevents blocking
  PubMultiQueue pmq;
};
}  // namespace gazebo
#endif  // CA_GAZEBO_CREATE_CLIFF_PLUGIN_H
