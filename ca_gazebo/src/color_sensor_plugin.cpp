#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "ca_gazebo/color_sensor_plugin.h"
#include <ros/console.h>
#include <boost/algorithm/string.hpp>

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <string>
#include <std_msgs/Bool.h>

namespace gazebo
{
  // Register this plugin with the simulator
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
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    CameraPlugin::Load(_parent, _sdf);
    // copying from CameraPlugin into GazeboRosCameraUtils
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;
    this->sensor_color_ = _sdf-> Get<std::string>("sensorColor");
    boost::algorithm::to_lower(this->sensor_color_);
    this->publish_topic_name_ = _sdf-> Get<std::string>("publishTopicName");
    this->update_period_ = 1.0/(_sdf-> Get<double>("updateRate"));
    this->_pixel_threshold = _sdf-> Get<double>("detectionCoefficient");
    this->_sensorPublisher = _nh.advertise<std_msgs::Bool>(this->publish_topic_name_, 1);
    this->_threshold_tolerance = 10;
    GazeboRosCameraUtils::Load(_parent, _sdf);
    InitColorMap();
    GetColorRGB();
  }


  void GazeboRosColor::GetColorRGB()
  {
    this->_goal_color = this->colorValues[this->sensor_color_];
  }

  void GazeboRosColor::InitColorMap()
  {
    this->colorValues.insert( std::make_pair("yellow", std::vector<int>{255, 255, 20}));
    this->colorValues.insert( std::make_pair("white",  std::vector<int>{255, 255, 255}));
  }

  bool GazeboRosColor::IsColorPresent(std::vector<double> current_color)
  {
    auto lambda = [=](double value, double threshold){return (abs(threshold+this->_threshold_tolerance) > value && abs(threshold-this->_threshold_tolerance) < value);};

    //ToDo: Make this prettier
    return (lambda(current_color[0], this->_goal_color[0]) &&
            lambda(current_color[1], this->_goal_color[1]) && 
            lambda(current_color[2], this->_goal_color[2]) 
          ) ;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboRosColor::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
  {
    static int seq=0;
    std_msgs::Bool msg;
    std::vector<double> current_rgb(3,0);
    this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();


    common::Time cur_time = this->world_->SimTime();
    if (cur_time - this->last_update_time_ >= this->update_period_)
    {
      this->PutCameraData(_image);
      this->PublishCameraInfo();
      this->last_update_time_ = cur_time;

      int64_t goal_color = 0;

      for (int i=0; i<(_height*_width*3)-3 ; i+=current_rgb.size())
      {
        current_rgb[0] = _image[i];
        current_rgb[1] = _image[i+1];
        current_rgb[2] = _image[i+2];

        if(IsColorPresent(current_rgb))
          goal_color++;

      }

      msg.data = goal_color > this->_pixel_threshold;
      _sensorPublisher.publish(msg);
      seq++;
    }
  }
}
