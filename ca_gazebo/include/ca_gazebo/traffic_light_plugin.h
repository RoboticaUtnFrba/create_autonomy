#ifndef GAZEBO_TRAFFIC_LIGHT_PLUGIN_HH
#define GAZEBO_TRAFFIC_LIGHT_PLUGIN_HH

#include <memory>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/Node.hh>

namespace gazebo
{
    class GazeboTrafficLightPrivate;

    class TrafficLightState {
        
        enum class States{RED, YELLOW, GREEN};
        
    };

    class GazeboTrafficLight : public VisualPlugin
    {

        /// Class for implementing a light traffic plugin for Gazebo.
        
        public:

        /// \brief Constructor.
        GazeboTrafficLight();

        /// \brief Destructor.
        ~GazeboTrafficLight();

        // Documentation inherited
        virtual void Load(rendering::VisualPtr _visual,
            sdf::ElementPtr _sdf);

        void time_update_cb(ConstTimePtr &);

        void init_map();
  
        private:
        
        /// \brief Update the plugin once every iteration of simulation.
        void update();

        /// \brief Private data pointer
        std::unique_ptr<GazeboTrafficLightPrivate> data_ptr;

        transport::SubscriberPtr command_subscriber;
        std::string next_color_;
        common::Time current_time_;
        common::Time last_time_;
        common::Time red_time_;
        common::Time yellow_time_;
        common::Time green_time_;
        TrafficLightState curr_color_ = TrafficLightState::RED; // Initial state is red

        // Custom type for making the code easier to read
        typedef std::pair<ignition::math::Color, common::Time> ColorTime;

        std::map<TrafficLightState, ColorTime> state_map;

    }; // class GazeboTrafficLight

} // namespace gazebo

#endif //GAZEBO_TRAFFIC_LIGHT_PLUGIN_HH
