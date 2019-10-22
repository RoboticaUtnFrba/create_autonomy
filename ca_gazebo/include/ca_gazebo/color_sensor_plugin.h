#ifndef GAZEBO_ROS_LIGHT_SENSOR_HH
#define GAZEBO_ROS_LIGHT_SENSOR_HH

#include <string>

// library for processing camera data for gazebo / ros conversions
#include <gazebo/plugins/CameraPlugin.hh>

#include <gazebo_plugins/gazebo_ros_camera_utils.h>

namespace gazebo
{
  class GazeboRosLight : public CameraPlugin, GazeboRosCameraUtils
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosLight();

    /// \brief Destructor
    public: ~GazeboRosLight();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    public: void GetColorRGB();

    //Decides if a given color is present, given the goal color and its relations (greater than or lower than)
    public: bool IsColorPresent(std::vector<double>, std::vector<double>, std::vector<bool>);

    /// \brief Update the controller
    protected: virtual void OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format);

    ros::NodeHandle _nh;
    ros::Publisher _sensorPublisher;

    double _fov;
    double _range;
    double PIXEL_AMOUNT_THRESHOLD = 50;
    std::string publish_topic_name_;
    std::string sensor_color_;

    // Custom type for the (double vector, double bool) type
    using DoubleBool = std::pair<std::vector<double>, std::vector<bool>>;

    //Variable with information about RGB thresholds and type of comparison for each RGB value 
    //That is, the bool vector indicates if the current pixel should have greater or smaller value than the one in the double vector
    DoubleBool RGBGoal;

    std::vector<double> YELLOW_COLOR = {250,250,20};
    std::vector<bool> YELLOW_RELATIONS = {true,true,false};
    std::pair<std::string, DoubleBool> YELLOW_DATA = {std::string("yellow"), DoubleBool(YELLOW_COLOR, YELLOW_RELATIONS)};
    std::vector<double> WHITE_COLOR = {250,250,250};
    std::vector<bool> WHITE_RELATIONS = {true,true,true};
    std::pair<std::string, DoubleBool> WHITE_DATA = {std::string("white"), DoubleBool(WHITE_COLOR, WHITE_RELATIONS)};
    
    std::map< std::string, std::pair<std::vector<double>, std::vector<bool>>> colorValues = {YELLOW_DATA,WHITE_DATA};
  };
}
#endif
