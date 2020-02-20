/*
 * Copyright 2020 Lucas Scheinkerman
 */

#ifndef CA_GAZEBO_WORLD_TIME_PUBLISHER_H
#define CA_GAZEBO_WORLD_TIME_PUBLISHER_H

#include "gazebo/gazebo.hh"
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/msgs/msgs.hh"

#define REFTIME 0.05   // ~ 20Hz

namespace gazebo
{
/// \Class WorldTimePublisher WorldTimePublisher.hh
/// \brief Class for publishing simulation time when diffrence is bigger than REFTIME.
class WorldTimePublisher: public WorldPlugin
{
private :
    /// \brief Remembering previus simulation time.
    double previous_ref_time;

    /// \brief Copy of current simulation time.
    double tmp_time;

    /// \brief Pointer to the world.
    physics::WorldPtr world;

    /// \brief Standard connection pointer.
    event::ConnectionPtr update_connection;

    /// \brief Transport node pointer.
    gazebo::transport::NodePtr node;

    /// \brief Transport publisher pointer.
    gazebo::transport::PublisherPtr pub;

    /// \brief Standard double vector message.
    gazebo::msgs::Time msg;

public:
    /// \brief Standard Load.
    void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);

    /// \brief Standard Init.
    void Init();

    /// \brief Update function for publishing messages on a topic.
    void OnUpdate();
};
}  // namespace gazebo

#endif  // CA_GAZEBO_WORLD_TIME_PUBLISHER_H
