/*
 * Copyright 2020
 *     Lucas Scheinkerman
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: Traffic Light Plugin
 * Author: Lucas Scheinkerman
 * Date: 19 February 2020
 */

// Gazebo libraries
#include <gazebo/common/Color.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/Node.hh>

// Ignition libraries
#include <ignition/math/Color.hh>

// ROS libraries
#include <ros/console.h>

// Project libraries
#include "ca_gazebo/traffic_light_plugin.h"

namespace gazebo
{
GZ_REGISTER_VISUAL_PLUGIN(GazeboTrafficLight)

TrafficLightState::TrafficLightState()
{}

TrafficLightState::~TrafficLightState()
{}

void TrafficLightState::next_state()
{
    switch (st)
    {
        case States::RED:
            st = States::YELLOW;
            break;
        case States::YELLOW:
            st = States::GREEN;
            break;
        case States::GREEN:
            st = States::RED;
            break;
    }
}

TrafficLightState::States TrafficLightState::get_current_state()
{
    return st;
}


class GazeboTrafficLightPrivate
{
public:
    /// Visual whose color will be changed.
    rendering::VisualPtr visual;

    /// Connects to rendering update event.
    event::ConnectionPtr updateConnection;

    /// Time on for every color.
    common::Time red_time_    = common::Time(15);
    common::Time yellow_time_ = common::Time(2);
    common::Time green_time_  = common::Time(6);

    /// Time the current cycle started.
    common::Time cycleStartTime;

    /// The current simulation time.
    common::Time currentSimTime;

    /// Node used for communication.
    transport::NodePtr node;

    /// Node used for communication.
    std::mutex mutex;

    /// Subscriber to world info.
    transport::SubscriberPtr infoSub;
};

GazeboTrafficLight::GazeboTrafficLight() :
data_ptr(new GazeboTrafficLightPrivate)
{
    init_map();
}

GazeboTrafficLight::~GazeboTrafficLight()
{}

void GazeboTrafficLight::init_map()
{
    ColorTime red_data = std::make_pair(
        ignition::math::Color(1, 0, 0), this->data_ptr->red_time_);
    ColorTime yellow_data = std::make_pair(
        ignition::math::Color(1, 1, 0), this->data_ptr->yellow_time_);
    ColorTime green_data = std::make_pair(
        ignition::math::Color(0, 1, 0), this->data_ptr->green_time_);

    this-> state_map =
    {
        {TrafficLightState::States::RED, red_data},
        {TrafficLightState::States::YELLOW, yellow_data},
        {TrafficLightState::States::GREEN, green_data},
    };
}

void GazeboTrafficLight::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
    // Check for null pointer
    if (!_visual || !_sdf)
    {
    gzerr << "No visual or SDF element specified. Plugin won't load." <<
        std::endl;
    return;
    }
    // Get parameters
    this->data_ptr->visual = _visual;
    this->data_ptr->red_time_    = common::Time(_sdf->Get<double>("redTime"));
    this->data_ptr->yellow_time_ = common::Time(_sdf->Get<double>("yellowTime"));
    this->data_ptr->green_time_  = common::Time(_sdf->Get<double>("greenTime"));
    // Check for correct time for every color
    if (this->data_ptr->red_time_ <= 0 || this->data_ptr->yellow_time_ <= 0 || this->data_ptr->green_time_ <= 0)
    {
        gzerr << "Time can't be lower than zero." << std::endl;
        return;
    }
    // Connect to the world update signal
    this->data_ptr->updateConnection =
        event::Events::ConnectPreRender(std::bind(&GazeboTrafficLight::update, this));
    // Subscribe to world statistics to get sim time
    // Warning: topic ~/pose/local/info is meant for high-bandwidth local
    // network access. It will kill the system if a remote gzclient tries to
    // subscribe.
    // Create our node for communication
    this->data_ptr->node = transport::NodePtr(new transport::Node());
    // Initialize the node with the world name
    this->data_ptr->node->Init();
    // Listen to Gazebo world_stats topic
    command_subscriber = this->data_ptr->node->Subscribe(
        "/gazebo/EmptyWorld/WorldTime_topic", &GazeboTrafficLight::time_update_cb, this);
    this->last_time_ = common::Time(0);
}

void GazeboTrafficLight::update()
{
    if (!this->data_ptr->visual)
    {
        gzerr << "The visual is null." << std::endl;
        return;
    }

    common::Time period_time = (this->state_map[this->curr_color_.get_current_state()]).second;

    const std::lock_guard<std::mutex> lock(this->data_ptr->mutex);

    const auto& elapsed = this->current_time_ - this->last_time_;

    if (elapsed >= period_time)
    {
        this->last_time_ = this->current_time_;

        ignition::math::Color color = (this->state_map[this->curr_color_.get_current_state()]).first;
        this->curr_color_.next_state();

        this->data_ptr->visual->SetDiffuse(color);
        this->data_ptr->visual->SetAmbient(color);
        this->data_ptr->visual->SetSpecular(color);
        this->data_ptr->visual->SetTransparency(0);
    }
}

void GazeboTrafficLight::time_update_cb(ConstTimePtr &_msg)
{
    const std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
    this->current_time_ = _msg->sec();
}

};  // namespace gazebo

