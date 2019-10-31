#include "ca_gazebo/world_time_publisher.h"

using namespace gazebo;

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
    tmp_time = this->world->SimTime().Double() ;
    if ( (tmp_time- previous_ref_time) <= REFTIME )
        return;
    previous_ref_time = tmp_time;
    gazebo::msgs::Set(&msg, this->world->SimTime());
    pub->Publish(msg);
}

GZ_REGISTER_WORLD_PLUGIN(WorldTimePublisher)