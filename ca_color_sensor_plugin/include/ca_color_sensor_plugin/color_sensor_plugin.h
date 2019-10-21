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
        std::string publish_topic_name_;
        //Decides each RGB value for a given color and its corresponding relation (greater than or lower than) according to the color sensor.
        std::string sensor_color_;
        std::vector<double> RGB_thresholds = std::vector<double>(3);
        //True = Greater than            False = Lower than
        std::vector<bool> RGB_relations = std::vector<bool>(3);

        std::vector<double> YELLOW_COLOR = {250,250,20};
        std::vector<bool> YELLOW_RELATIONS = {true,true,false};
        std::vector<double> WHITE_COLOR = {250,250,250};
        std::vector<bool> WHITE_RELATIONS = {true,true,true};


      };
    }
    #endif
