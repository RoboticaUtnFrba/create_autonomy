#include "ca_gazebo/world_time_publisher.h"

using namespace gazebo;

//////////////////////////////////////////////////
void WorldTimePublisher::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
{	std::cout << "WorldPublisher: Entering Load()" << std::endl;
    this->world = _parent;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&WorldTimePublisher::OnUpdate, this));
    PreviusRefTime = 0;
}

////////////////////////////////////////////////
void WorldTimePublisher::Init()
 {	std::cout << "WorldPublisher: Entering Init()" << std::endl;
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init("EmptyWorld");
    this->pub = node->Advertise<gazebo::msgs::Time>("~/WorldTime_topic");
 }

//////////////////////////////////////////////////
void WorldTimePublisher::OnUpdate()
{
    //std::cout << "WorldPublisher: Entering OnUpdate()" << std::endl;
    tmpTime = this->world->GetSimTime().Double() ;
    if ( (tmpTime- PreviusRefTime) <= REFTIME )
        return;
    PreviusRefTime = tmpTime;
    //msg.set_sec(this->world->GetSimTime().sec);
    //msg.set_nsec(this->world->GetSimTime().nsec);
    gazebo::msgs::Set(&msg, this->world->GetSimTime());
    pub->Publish(msg);
}

GZ_REGISTER_WORLD_PLUGIN(WorldTimePublisher)