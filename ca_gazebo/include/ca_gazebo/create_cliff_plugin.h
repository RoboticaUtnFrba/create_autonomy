#ifndef GAZEBO_ROS_CLIFF_SENSOR_HH
#define GAZEBO_ROS_CLIFF_SENSOR_HH

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

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
    /// \brief Constructor
    public: GazeboRosCliff();

    /// \brief Destructor
    public: ~GazeboRosCliff();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Keep track of number of connctions
    private: int laser_connect_count_;
    private: void LaserConnect();
    private: void LaserDisconnect();

    // Pointer to the model
    private: std::string world_name_;
    private: physics::WorldPtr world_;
    /// \brief The parent sensor
    private: sensors::RaySensorPtr parent_ray_sensor_;

    /// \brief pointer to ros node
    private: std::shared_ptr<ros::NodeHandle> rosnode_;
    private: ros::Publisher pub_;
    private: PubQueue<std_msgs::Bool>::Ptr pub_queue_;

    /// \brief topic name
    private: std::string topic_name_;

    /// \brief frame transform name, should match link name
    private: std::string frame_name_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;
    private: unsigned int seed;

    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr laser_scan_sub_;
    private: void OnScan(ConstLaserScanStampedPtr &_msg);

    /// Auxiliar variables to get the info from the Ray Sensor
    private: float min_cliff_value;

    /// \brief prevents blocking
    private: PubMultiQueue pmq;
  };
}
#endif
