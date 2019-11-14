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
    nh_("color_sensor_plugin"),
    fov_(6),
    range_(10),
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
    GazeboRosCameraUtils::Load(_parent, _sdf);
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->format_ = this->format;
    this->camera_ = this->camera;
    this->sensor_color_ = _sdf->Get<std::string>("sensorColor");
    boost::algorithm::to_lower(this->sensor_color_);
    std::string publish_topic_name_ = _sdf->Get<std::string>("publishTopicName");
    this->update_period_ = 1.0/(_sdf->Get<double>("updateRate"));
    this->pixel_threshold_ = _sdf->Get<double>("detectionCoefficient");
    this->sensor_publisher_ = nh_.advertise<std_msgs::Bool>(publish_topic_name_, 1);
    this->threshold_tolerance_ = 10;

    GetColorRGB();

    this->parentSensor_->SetActive(true);
    this->last_update_time_ = this->world_->SimTime();
  }

  void GazeboRosColor::GetColorRGB()
  {
    this->goal_color_ = this->color_values_.at(this->sensor_color_);
  }

  bool GazeboRosColor::IsColorPresent(std::vector<double>& current_color)
  {
    // This lambda function compares, one at a time, each of the RGB values of the parameter of this function with the target RGB colors.
    auto compare_element = [=](double value, double threshold)
    {
      return (abs(threshold+this->threshold_tolerance_) > value   &&   abs(threshold-this->threshold_tolerance_) < value);
    };

    //ToDo: Make this prettier
    return (compare_element(current_color[0], this->goal_color_[0]) &&
            compare_element(current_color[1], this->goal_color_[1]) && 
            compare_element(current_color[2], this->goal_color_[2]) 
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

      double pixel_count = 0;

      for (int i = 0; i < (_height*_width*3)-3 ; i += current_rgb.size())
      {
        current_rgb[0] = _image[i];
        current_rgb[1] = _image[i+1];
        current_rgb[2] = _image[i+2];

        if(IsColorPresent(current_rgb))
          pixel_count++;
      }

      std_msgs::BoolPtr msg(new std_msgs::Bool);
      msg->data = pixel_count > this->pixel_threshold_;
      sensor_publisher_.publish(msg);
    }
  }
} // namespace gazebo
