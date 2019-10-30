#ifndef _WORLD_TIME_PUBLISHER_HH
#define _WORLD_TIME_PUBLISHER_HH

#include "gazebo/gazebo.hh"
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/msgs/msgs.hh"

//#define REFTIME 0.1 // ~ 10Hz
#define REFTIME 0.05 // ~ 20Hz

namespace gazebo {

     /// \Class WorldTimePublisher WorldTimePublisher.hh
    /// \brief Class for publishing simulation time when diffrence is bigger than REFTIME.
    class WorldTimePublisher: public WorldPlugin
    {
        /// \brief Remembering previus simulation time.
        private : double PreviusRefTime;

        /// \brief Copy of current simulation time.
        private : double tmpTime;

        /// \brief Pointer to the world.
        private: physics::WorldPtr world;

        /// \brief Standard connection pointer.
        private: event::ConnectionPtr updateConnection;

        /// \brief Transport node pointer.
        private: gazebo::transport::NodePtr node;

        /// \brief Transport publisher pointer.
        gazebo::transport::PublisherPtr pub;

        /// \brief Standard double vector message.
        gazebo::msgs::Time msg;

        /// \brief Standard Load.
        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);

        /// \brief Standard Init.
        public: void Init();

        /// \brief Update function for publishing messages on a topic.
        public: void OnUpdate();
    };
}

#endif