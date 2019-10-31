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

    class GazeboTrafficLight : public VisualPlugin
    {

        /// \brief Constructor.
        public: GazeboTrafficLight();

        /// \brief Destructor.
        public: ~GazeboTrafficLight();

        // Documentation inherited
        public: virtual void Load(rendering::VisualPtr _visual,
            sdf::ElementPtr _sdf);

        public: void time_update_cb(ConstTimePtr &);

        /// \brief Update the plugin once every iteration of simulation.
        private: void Update();

        /// \internal
        /// \brief Private data pointer
        private: std::unique_ptr<GazeboTrafficLightPrivate> data_ptr;

        transport::SubscriberPtr commandSubscriber;
        std::string next_color_;
        common::Time current_time_;
        common::Time last_time_;
        common::Time red_time_;
        common::Time yellow_time_;
        common::Time green_time_;
    };
}
#endif
