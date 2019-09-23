    #ifndef GAZEBO_VIRTUAL_WALL_DETECTOR_HH
    #define GAZEBO_VIRTUAL_WALL_DETECTOR_HH
    
    #include <ros/ros.h>
    
    #include <gazebo/gazebo.hh>
    #include <gazebo/physics/physics.hh>
    #include <gazebo/common/common.hh>
    #include <gazebo/common/Plugin.hh>

    #include <mutex>

    #include "std_msgs/Bool.h"

    namespace gazebo
    {
        /// \brief A Virtual Wall publisher
        class GazeboVirtualWallDetector : public ModelPlugin
    {
        public:

        /// \brief Load the plugin
        /// \param take in SDF root element
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf); 

        /// \brief Callback to the virtual wall sensors
        /// \param take pointer to the message
        void WallCallback(const std_msgs::Bool::ConstPtr& data); 

        /// \brief Update the controller
        void OnUpdate();

        private:

        /// Pointer to the model
        physics::ModelPtr model;

        /// Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;

        /// Initialize ROS variables
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        ros::Subscriber subscriber_;
        ros::Time prev_update_time_;
        event::ConnectionPtr updateConnection_;
        
        /// Auxiliar variables
        bool is_vwall_detected_;
        bool get_vwall_;
        double update_period_;
        std::mutex vwall_mutex;
      }; // GazeboVirtualWallDetector
    } // gazebo
    #endif // GAZEBO_VIRTUAL_WALL_DETECTOR_HH
