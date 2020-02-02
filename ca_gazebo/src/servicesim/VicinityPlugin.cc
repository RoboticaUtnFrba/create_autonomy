/*
 * Copyright (C) 2018 Open Source Robotics Foundation, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "VicinityPlugin.hh"

using namespace servicesim;

GZ_REGISTER_MODEL_PLUGIN(servicesim::VicinityPlugin);

//////////////////////////////////////////////////
VicinityPlugin::VicinityPlugin()
{
}

//////////////////////////////////////////////////
VicinityPlugin::~VicinityPlugin()
{

}

//////////////////////////////////////////////////
void VicinityPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->rosnode_ = new ros::NodeHandle("/" + _parent->GetName());

  this->model_ = _parent;
  this->world_ = _parent->GetWorld(); // Store the pointer to the world

  for (unsigned int i = 0; i < this->world_->ModelCount(); ++i)
  {
    auto model = this->world_->ModelByIndex(i);
    if (gazebo::physics::ActorPtr a =
        boost::dynamic_pointer_cast<gazebo::physics::Actor>(model))
    {
      this->actorPtrs_.push_back(a);
    }
  }
  gzmsg << "[VicinityPlugin] Found " << this->actorPtrs_.size()
        << " actors in the world" << std::endl;

  if (!_sdf->HasElement("threshold"))
  {
    this->threshold_ = 5.0;
  }
  else
  {
    this->threshold_ = _sdf->GetElement("threshold")->Get<double>();
  }

  if (!_sdf->HasElement("topicName"))
  {
    this->topicName_ = "RFID";
  }
  else
  {
    this->topicName_ = _sdf->GetElement("topicName")->Get<std::string>();
  }

  if (!_sdf->HasElement("updateRate"))
  {
    this->update_rate_ = 1.0;
  }
  else
  {
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
  }

  this->vicinity_pub_ =
      this->rosnode_->advertise<ca_msgs::ActorNames>(
      this->topicName_, 1
  );


  this->last_time_ = this->world_->SimTime();

  // Listen to the update event
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&VicinityPlugin::Update, this));
}

//////////////////////////////////////////////////
void VicinityPlugin::Update()
{
  gazebo::common::Time cur_time = this->world_->SimTime();

  if (this->update_rate_ > 0 &&
      (cur_time - this->last_time_).Double() < (1.0 / this->update_rate_))
    return;

  ca_msgs::ActorNames msg;
  for (auto actor : actorPtrs_)
  {
    auto robotPos = this->model_->WorldPose().Pos();
    auto actorPos = actor->WorldPose().Pos();

    auto poseDiff = actorPos - robotPos;
    poseDiff.Z(0);
    if (poseDiff.Length() <= this->threshold_)
    {
      msg.actor_names.push_back(actor->GetName());
    }
  }
  if (msg.actor_names.size() > 0)
  {
    this->vicinity_pub_.publish(msg);
  }
  this->last_time_ = cur_time;
}
