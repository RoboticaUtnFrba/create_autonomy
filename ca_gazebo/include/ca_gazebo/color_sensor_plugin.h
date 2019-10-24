#ifndef GAZEBO_ROS_COLOR_SENSOR_HH
#define GAZEBO_ROS_COLOR_SENSOR_HH

#include <string>

// library for processing camera data for gazebo / ros conversions
#include <gazebo/plugins/CameraPlugin.hh>

#include <gazebo_plugins/gazebo_ros_camera_utils.h>

namespace gazebo
{
  class GazeboRosColor : public CameraPlugin, GazeboRosCameraUtils
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosColor();

    /// \brief Destructor
    public: ~GazeboRosColor();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    public: void GetColorRGB();

    //Decides if a given color is present, given the goal color
    public: bool IsColorPresent(std::vector<double>);

    public: void InitColorMap();

    /// \brief Update the controller
    protected: virtual void OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format);

    ros::NodeHandle _nh;
    ros::Publisher _sensorPublisher;
    ros::Publisher colorpub;

    double _fov;
    double _range;
    double _pixel_threshold;
    double _threshold_tolerance;
    std::string publish_topic_name_;
    std::string sensor_color_;

    //Variable with information about RGB threshold values
    std::vector<int> _goal_color;
    std::map<std::string, std::vector<int>> colorValues;
  };
}
#endif
