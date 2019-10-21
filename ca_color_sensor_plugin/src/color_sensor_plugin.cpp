#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "ca_color_sensor_plugin/color_sensor_plugin.h"

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sensor_msgs/Illuminance.h>
#include <std_msgs/Bool.h>

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
  CameraPlugin()
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
    // copying from CameraPlugin into GazeboRosCameraUtils
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;
    this->publish_topic_name_ = _sdf-> Get<std::string>("publishTopicName");
    this->sensor_color_ = _sdf-> Get<std::string>("sensorColor");
    _sensorPublisher = _nh.advertise<std_msgs::Bool>(this->publish_topic_name_, 1);

    GazeboRosCameraUtils::Load(_parent, _sdf);

  }

  void GazeboRosLight::GetColorRGB()
  {
    //ToDo: Make this prettier with a dictionary.
    if(this->sensor_color_ == std::string("Yellow"))
    {
      this->RGB_thresholds = this->YELLOW_COLOR;
      this->RGB_relations = this->YELLOW_RELATIONS;
    }
    else if(this->sensor_color_ == std::string("White"))
    {
      this->RGB_thresholds = this->WHITE_COLOR;
      this->RGB_relations = this->WHITE_RELATIONS;
    }
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
    GetColorRGB();
    //this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();


    if (!this->parentSensor->IsActive())
    {
      if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run once after activated
        this->parentSensor->SetActive(true);
    }
    else
    {
      if ((*this->image_connect_count_) > 0)
      {
        common::Time cur_time = this->world_->SimTime();
        if (cur_time - this->last_update_time_ >= this->update_period_)
        {
          this->PutCameraData(_image);
          this->PublishCameraInfo();
          this->last_update_time_ = cur_time;

          std_msgs::Bool msg;

          std::vector<double> current_rgb = std::vector<double>(3);

          double goal_color = 0;

          for (int i=0; i<(_height*_width)-2 ; ++i)
          {
            current_rgb[0] = _image[i];
            i++;
            current_rgb[1] = _image[i];
            i++;
            current_rgb[2] = _image[i];

            if(IsColorPresent(this->RGB_thresholds, current_rgb, this->RGB_relations) )
              goal_color++;
          }


          msg.data = (goal_color > 100) ; //hardcode
          _sensorPublisher.publish(msg);

          seq++;
        }
      }
    }
  }
}
