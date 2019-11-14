#ifndef GAZEBO_ROS_COLOR_SENSOR_HH
#define GAZEBO_ROS_COLOR_SENSOR_HH

// C++ libraries
#include <string>

// Gazebo libraries
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>

namespace gazebo
{
  class GazeboRosColor : public CameraPlugin, GazeboRosCameraUtils
  {
    public: 
    
    // Constructor
    GazeboRosColor();

    // Destructor
    ~GazeboRosColor();

    // Load the plugin. The load function is called by Gazebo when the plugin is inserted into simulation.
    // _parent is the parent entity, must be a Model or a Sensor.
    // _sdf is the SDF which invokes this plugin, which defines parameters for this plugin.
    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    void GetColorRGB();

    // Decides if a given color is present, given the goal color.
    // The input parameter is the RGB vector of a pixel, which is compared with the class attribute 'colorValues'.
    bool IsColorPresent(std::vector<double>&);

    protected: 
    
    // Proccesses each new frame given by Gazebo for detecting if the target color is present.
    virtual void OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format);

    ros::NodeHandle nh_;
    ros::Publisher sensor_publisher_;

    double fov_;
    double range_;
    double pixel_threshold_;
    double threshold_tolerance_;
    std::string sensor_color_;

    // Variable with information about RGB threshold values depending on the target color of the sensor.
    std::vector<int> goal_color_;
    const std::map<std::string, std::vector<int>> color_values_={
                                                                {"yellow",{255,255,20}},
                                                                {"white",{255,255,255}}
    };
  };
}
#endif //GAZEBO_ROS_COLOR_SENSOR_HH
