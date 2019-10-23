#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "ca_gazebo/color_sensor_plugin.h"
#include <ros/console.h>
#include <boost/algorithm/string.hpp>

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <string>
#include <std_msgs/Bool.h>
#include <stdlib.h>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLight)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  GazeboRosLight::GazeboRosLight():
  _nh("color_sensor_plugin"),
  _fov(6),
  _range(10),
  CameraPlugin(),
  GazeboRosCameraUtils()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboRosLight::~GazeboRosLight()
  {
    ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
  }

  void GazeboRosLight::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    CameraPlugin::Load(_parent, _sdf);
    //copying from CameraPlugin into GazeboRosCameraUtils
    GazeboRosCameraUtils::Load(_parent, _sdf);

    this->parentSensor_ = _parent;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;
    this->publish_topic_name_ = _sdf-> Get<std::string>("publishTopicName");
    this->sensor_color_ = _sdf-> Get<std::string>("sensorColor");
    this->update_period_ = 1.0/(atof(_sdf-> Get<std::string>("update_rate").c_str()));
    this->pixel_threshold_ = atof(_sdf-> Get<std::string>("detectionCoefficient").c_str());
    //boost::algorithm::to_upper(this->sensor_color_);
    _sensorPublisher = _nh.advertise<std_msgs::Bool>(this->publish_topic_name_, 1);
    GetColorRGB();

    this->parentSensor_->SetActive(true);

  }

  void GazeboRosLight::GetColorRGB()
  {
    this->RGBGoal = this->colorValues[this->sensor_color_];
  }

  bool GazeboRosLight::IsColorPresent(std::vector<double> goal_color, std::vector<double> current_color, std::vector<bool> relations)
  {
    auto lambda = [=](double a, double b, bool r){ return (r? a>b : a<b); };

    //ToDo: Make this prettier
    return ( lambda(current_color[0], goal_color[0], relations[0]) && lambda(current_color[1], goal_color[1], relations[1]) && lambda(current_color[2], goal_color[2], relations[2]) ) ;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboRosLight::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
  {
    static int seq=0;
    this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();
    std_msgs::Bool msg;
    std::vector<double> current_rgb = std::vector<double>(3);


    common::Time cur_time = this->world_->SimTime();
    if (cur_time - this->last_update_time_ >= this->update_period_)
    {

      double goal_color = 0;

      for (int i=0; i<(_height*_width)-2 ; i+=current_rgb.size())
      {
        current_rgb[0] = _image[i];
        current_rgb[1] = _image[i+1];
        current_rgb[2] = _image[i+2];

        if(IsColorPresent(this->RGBGoal.first, current_rgb, this->RGBGoal.second) )
          goal_color++;
      }

      msg.data = (goal_color > this->pixel_threshold_);
      _sensorPublisher.publish(msg);

      seq++;
      this->PutCameraData(_image);
      this->PublishCameraInfo();
      this->last_update_time_ = cur_time;
    }
  }
}
