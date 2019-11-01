// C++ libraries
#include <string>
#include <boost/algorithm/string.hpp>

// ROS libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>

// Gazebo libraries
#include "gazebo_plugins/gazebo_ros_camera.h"
#include <gazebo/common/Plugin.hh>

// Project libraries
#include "ca_gazebo/color_sensor_plugin.h"


namespace gazebo
{
  // Register this plugin with the simulator.
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosColor)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  GazeboRosColor::GazeboRosColor():
    _nh("color_sensor_plugin"),
    _fov(6),
    _range(10),
    CameraPlugin(),
    GazeboRosCameraUtils()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboRosColor::~GazeboRosColor()
  {
    ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
  }

  void GazeboRosColor::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized.
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    CameraPlugin::Load(_parent, _sdf);
    // Copying from CameraPlugin into GazeboRosCameraUtils.
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->format_ = this->format;
    this->camera_ = this->camera;
    this->sensor_color_ = _sdf->Get<std::string>("sensorColor");
    boost::algorithm::to_lower(this->sensor_color_);
    std::string publish_topic_name_ = _sdf->Get<std::string>("publishTopicName");
    this->update_period_ = 1.0/(_sdf->Get<double>("updateRate"));
    this->_pixel_threshold = _sdf->Get<double>("detectionCoefficient");
    this->_sensorPublisher = _nh.advertise<std_msgs::Bool>(publish_topic_name_, 1);
    this->_threshold_tolerance = 10;
    GazeboRosCameraUtils::Load(_parent, _sdf);
    GetColorRGB();

    this->parentSensor_->SetActive(true);
  }

  void GazeboRosColor::GetColorRGB()
  {
    this->_goal_color = this->colorValues.at(this->sensor_color_);
  }

  bool GazeboRosColor::IsColorPresent(std::vector<double>& current_color)
  {
    // This lambda function compares, one at a time, each of the RGB values of the parameter of this function with the target RGB colors.
    auto compare_element = [=](double value, double threshold)
    {
      return (abs(threshold+this->_threshold_tolerance) > value   &&   abs(threshold-this->_threshold_tolerance) < value);
    };

    //ToDo: Make this prettier
    return (compare_element(current_color[0], this->_goal_color[0]) &&
            compare_element(current_color[1], this->_goal_color[1]) && 
            compare_element(current_color[2], this->_goal_color[2]) 
          ) ;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Proccesses each new frame given by Gazebo for detecting if the target color is present.
  void GazeboRosColor::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth, 
    const std::string &_format)
  {
    const common::Time cur_time = this->world_->SimTime();
    if (cur_time - this->last_update_time_ >= this->update_period_)
    {
      this->last_update_time_ = cur_time;
      std::vector<double> current_rgb(3,0);

      double goal_color = 0;

      for (int i = 0; i < (_height*_width*3)-3 ; i += current_rgb.size())
      {
        current_rgb[0] = _image[i];
        current_rgb[1] = _image[i+1];
        current_rgb[2] = _image[i+2];

        if(IsColorPresent(current_rgb))
          goal_color++;
      }

      std_msgs::Bool msg;
      msg.data = goal_color > this->_pixel_threshold;
      _sensorPublisher.publish(msg);
    }
  }
}
