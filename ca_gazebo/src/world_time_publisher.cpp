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

#include "ca_gazebo/world_time_publisher.h"

namespace gazebo
{
//////////////////////////////////////////////////
void WorldTimePublisher::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    this->world = _parent;
    this->update_connection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&WorldTimePublisher::OnUpdate, this));
    previous_ref_time = 0;
}

////////////////////////////////////////////////
void WorldTimePublisher::Init()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("EmptyWorld");
  this->pub = node->Advertise<gazebo::msgs::Time>("~/WorldTime_topic");
}

//////////////////////////////////////////////////
void WorldTimePublisher::OnUpdate()
{
    tmp_time = this->world->SimTime().Double();
    if ( (tmp_time- previous_ref_time) <= REFTIME )
        return;
    previous_ref_time = tmp_time;
    gazebo::msgs::Set(&msg, this->world->SimTime());
    pub->Publish(msg);
}

GZ_REGISTER_WORLD_PLUGIN(WorldTimePublisher)
};  // namespace gazebo
