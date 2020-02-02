/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <functional>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/msgs/boolean.pb.h>
#include <ignition/transport/Node.hh>

#include <gazebo/common/Animation.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/KeyFrame.hh>
#include <gazebo/physics/physics.hh>

#include "FollowActorPlugin.hh"

#include <ros/ros.h>

using namespace servicesim;
GZ_REGISTER_MODEL_PLUGIN(servicesim::FollowActorPlugin)

class servicesim::FollowActorPluginPrivate
{
  /// \brief Pointer to the actor.
  public: gazebo::physics::ActorPtr actor{nullptr};

  /// \brief Velocity of the actor
  public: double velocity{0.8};

  /// \brief List of connections such as WorldUpdateBegin
  public: std::vector<gazebo::event::ConnectionPtr> connections;

  /// \brief Current target model to follow
  public: gazebo::physics::ModelPtr target{nullptr};

  /// \brief Minimum distance in meters to keep away from target.
  public: double minDistance{1.2};

  /// \brief Maximum distance in meters to keep away from target.
  public: double maxDistance{4};

  /// \brief Radius around actor from where it can be picked up
  public: double pickUpRadius{2};

  /// \brief Margin by which to increase an obstacle's bounding box on every
  /// direction (2x per axis).
  public: double obstacleMargin{0.5};

  /// \brief Time scaling factor. Used to coordinate translational motion
  /// with the actor's walking animation.
  public: double animationFactor{5.1};

  /// \brief Maximum angle when drifting
  public: double maxDriftAngle{IGN_PI * 0.2};

  /// \brief List of times when actor should drift away
  public: std::vector<gazebo::common::Time> driftTimes;

  /// \brief Time of the last update.
  public: gazebo::common::Time lastUpdate;

  /// \brief Time tolerance when checking drift time
  public: gazebo::common::Time timeTolerance{0.5};

  /// \brief List of models to ignore when checking collisions.
  public: std::vector<std::string> ignoreModels;

  /// \brief Ignition transport node for communication
  public: ignition::transport::Node ignNode;

  /// \brief Publishes drift notifications
  public: ignition::transport::Node::Publisher driftIgnPub;

  /// \brief Namespace for Ignition transport communication:
  /// * /<namespace>/<actor_name>/follow
  /// * /<namespace>/<actor_name>/unfollow
  /// * /<namespace>/<actor_name>/drift
  public: std::string ns;

  /// \brief ROS node handle
  public: ros::NodeHandle rosNode;

  /// \brief ROS drift cheat service server
  public: ros::ServiceServer driftService;

  /// \brief Flag to enable drift when requested via ROS
  public: bool driftFlag = false;
};

/////////////////////////////////////////////////
FollowActorPlugin::FollowActorPlugin()
    : dataPtr(new FollowActorPluginPrivate)
{
}

/////////////////////////////////////////////////
void FollowActorPlugin::Load(gazebo::physics::ModelPtr _model,
    sdf::ElementPtr _sdf)
{
  this->dataPtr->actor =
      boost::dynamic_pointer_cast<gazebo::physics::Actor>(_model);

  // Read in the namespace
  if (_sdf->HasElement("namespace"))
    this->dataPtr->ns = "/" + _sdf->Get<std::string>("namespace");

  // Read in the velocity
  if (_sdf->HasElement("velocity"))
    this->dataPtr->velocity = _sdf->Get<double>("velocity");

  // Read in the follow distance
  if (_sdf->HasElement("min_distance"))
    this->dataPtr->minDistance = _sdf->Get<double>("min_distance");

  // Read in the follow distance
  if (_sdf->HasElement("max_distance"))
    this->dataPtr->maxDistance = _sdf->Get<double>("max_distance");

  // Read in the pickup radius
  if (_sdf->HasElement("pickup_radius"))
    this->dataPtr->pickUpRadius = _sdf->Get<double>("pickup_radius");

  // Read in the obstacle margin
  if (_sdf->HasElement("obstacle_margin"))
    this->dataPtr->obstacleMargin = _sdf->Get<double>("obstacle_margin");

  // Read in the animation factor
  if (_sdf->HasElement("animation_factor"))
    this->dataPtr->animationFactor = _sdf->Get<double>("animation_factor");

  // Add our own name to models we should ignore when avoiding obstacles.
  this->dataPtr->ignoreModels.push_back(this->dataPtr->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacle"))
  {
    auto ignoreElem = _sdf->GetElement("ignore_obstacle");
    while (ignoreElem)
    {
      this->dataPtr->ignoreModels.push_back(ignoreElem->Get<std::string>());
      ignoreElem = ignoreElem->GetNextElement("ignore_obstacle");
    }
  }

  // Read in drift times
  if (_sdf->HasElement("drift_time"))
  {
    auto driftElem = _sdf->GetElement("drift_time");
    while (driftElem)
    {
      this->dataPtr->driftTimes.push_back(driftElem->Get<double>());
      driftElem = driftElem->GetNextElement("drift_time");
    }
  }

  // Read in the animation name
  std::string animation{"animation"};
  if (_sdf->HasElement("animation"))
    animation = _sdf->Get<std::string>("animation");

  auto skelAnims = this->dataPtr->actor->SkeletonAnimations();
  if (skelAnims.find(animation) == skelAnims.end())
  {
    gzerr << "Skeleton animation [" << animation << "] not found in Actor."
          << std::endl;
  }
  else
  {
    // Set custom trajectory
    gazebo::physics::TrajectoryInfoPtr
        trajectoryInfo(new gazebo::physics::TrajectoryInfo());
    trajectoryInfo->type = animation;
    trajectoryInfo->duration = 1.0;

    this->dataPtr->actor->SetCustomTrajectory(trajectoryInfo);
  }

  // Update loop
  this->dataPtr->connections.push_back(
      gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&FollowActorPlugin::OnUpdate, this, std::placeholders::_1)));

  // Pickup service
  const std::string followSrvName = this->dataPtr->ns + "/" + this->dataPtr->actor->GetName() + "/follow";
  this->dataPtr->ignNode.Advertise(
      followSrvName, &FollowActorPlugin::OnFollow, this);

  const std::string unfollowSrvName = this->dataPtr->ns + "/" + this->dataPtr->actor->GetName() + "/unfollow";
  this->dataPtr->ignNode.Advertise(
      unfollowSrvName, &FollowActorPlugin::OnUnfollow, this);

  // Drift publisher
  this->dataPtr->driftIgnPub =
      this->dataPtr->ignNode.Advertise<ignition::msgs::UInt32>(
      this->dataPtr->ns + "/" + this->dataPtr->actor->GetName() + "/drift");

  // Advertise drift cheat service
  this->dataPtr->driftService = this->dataPtr->rosNode.advertiseService(
      "/servicesim/drift", &FollowActorPlugin::OnDriftRosService, this);
}

/////////////////////////////////////////////////
void FollowActorPlugin::Reset()
{
  if (this->dataPtr->actor && this->dataPtr->target)
  {
    gzmsg << "Actor [" << this->dataPtr->actor->GetName()
          << "] stopped following target [" << this->dataPtr->target->GetName()
          << "]" << std::endl;
  }
  this->dataPtr->target = nullptr;
  this->dataPtr->lastUpdate = gazebo::common::Time::Zero;
}

/////////////////////////////////////////////////
bool FollowActorPlugin::ObstacleOnTheWay() const
{
  auto actorPose = this->dataPtr->actor->WorldPose().Pos();
  auto world = this->dataPtr->actor->GetWorld();

  // Iterate over all models in the world
  for (unsigned int i = 0; i < world->ModelCount(); ++i)
  {
    // Skip models we're ignoring
    auto model = world->ModelByIndex(i);
    if (std::find(this->dataPtr->ignoreModels.begin(),
                  this->dataPtr->ignoreModels.end(), model->GetName()) !=
                  this->dataPtr->ignoreModels.end())
    {
      continue;
    }

    // Obstacle's bounding box
    auto bb = model->BoundingBox();

    // Models without collision have invalid boxes
    if (!bb.Min().IsFinite() || !bb.Max().IsFinite())
      continue;

    // Increase box by margin
    bb.Min() -= ignition::math::Vector3d::One * this->dataPtr->obstacleMargin;
    bb.Max() += ignition::math::Vector3d::One * this->dataPtr->obstacleMargin;

    // Increase vertically
    bb.Min().Z() -= 5;
    bb.Max().Z() += 5;

    // Check
    if (bb.Contains(actorPose))
    {
      return true;
    }

    // TODO: Improve obstacle avoidance. Some ideas: check contacts, ray-query
    // the path forward, check against bounding box of each collision shape...
  }
  return false;
}

/////////////////////////////////////////////////
void FollowActorPlugin::OnUpdate(const gazebo::common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->dataPtr->lastUpdate).Double();

  this->dataPtr->lastUpdate = _info.simTime;

  // Is there a follow target?
  if (!this->dataPtr->target)
    return;

  // Don't move if there's an obstacle on the way
  // TODO: Find another way to handle obstacles
//  if (this->ObstacleOnTheWay())
  //  return;

  // Is it drift time?
  gazebo::common::Time driftTime;
  for (auto t : this->dataPtr->driftTimes)
  {
    double diff = abs((t - _info.simTime).Double());
    if (diff <= this->dataPtr->timeTolerance.Double())
    {
      driftTime = t;
      break;
    }

    if (t > _info.simTime)
      break;
  }

  // Current pose - actor is oriented Y-up and Z-front
  auto actorPose = this->dataPtr->actor->WorldPose();
  auto zPos = actorPose.Pos().Z();

  // Current target
  auto targetPose = this->dataPtr->target->WorldPose();

  // Direction to target
  auto dir = targetPose.Pos() - actorPose.Pos();
  dir.Z(0);

  // Stop if too close to target
  if (driftTime == gazebo::common::Time::Zero &&
      dir.Length() <= this->dataPtr->minDistance)
  {
    return;
  }

  // Stop following if too far from target
  if (dir.Length() > this->dataPtr->maxDistance)
  {
    gzwarn << "Target [" << this->dataPtr->target->GetName()
           <<  "] too far, actor [" << this->dataPtr->actor->GetName()
           <<"] stopped following" << std::endl;
    this->dataPtr->target = nullptr;

    // Publish drift notification
    // 1: target too far
    ignition::msgs::UInt32 msg;
    msg.set_data(1);
    this->dataPtr->driftIgnPub.Publish(msg);

    return;
  }

  dir.Normalize();

  // Towards target
  ignition::math::Angle yaw = atan2(dir.Y(), dir.X()) + IGN_PI_2;

  // Drift
  if (driftTime != gazebo::common::Time::Zero || this->dataPtr->driftFlag)
  {
    // Change direction a bit
    yaw += ignition::math::Rand::DblUniform(-1, 1) *
        this->dataPtr->maxDriftAngle;

    // Stop following
    this->dataPtr->target = nullptr;

    // Notify
    // 2: drift time
    ignition::msgs::UInt32 msg;
    msg.set_data(2);
    this->dataPtr->driftIgnPub.Publish(msg);

    if (!this->dataPtr->driftFlag)
    {
      gzwarn << "Actor [" << this->dataPtr->actor->GetName()
             <<  "] drifting due to scheduled time: " << driftTime << std::endl;
    }
    else
    {
      gzwarn << "Actor [" << this->dataPtr->actor->GetName()
             <<  "] drifted as requested! (cheat)" << std::endl;
      this->dataPtr->driftFlag = false;
    }
    // Don't return yet, so the actor moves away
  }
  yaw.Normalize();

  actorPose.Pos() += dir * this->dataPtr->velocity * dt;
  actorPose.Pos().Z(zPos);
  actorPose.Rot() = ignition::math::Quaterniond(IGN_PI_2, 0, yaw.Radian());

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (actorPose.Pos() -
      this->dataPtr->actor->WorldPose().Pos()).Length();

  // Update actor
  this->dataPtr->actor->SetWorldPose(actorPose, false, false);
  this->dataPtr->actor->SetScriptTime(this->dataPtr->actor->ScriptTime() +
    (distanceTraveled * this->dataPtr->animationFactor));
}

/////////////////////////////////////////////////
bool FollowActorPlugin::OnFollow(const ignition::msgs::StringMsg &_req,
    ignition::msgs::Boolean &_res)
{
  _res.set_data(false);
  
  // Get target model
  auto targetName = _req.data();

  auto world = this->dataPtr->actor->GetWorld();

  auto model = world->ModelByName(targetName);
  if (!model)
  {
    gzwarn << "Failed to find model: [" << targetName << "]" << std::endl;
    return false;
  }

  // Check pickup radius
  auto actorPos = this->dataPtr->actor->WorldPose().Pos();
  auto targetPos = model->WorldPose().Pos();

  auto posDiff = actorPos - targetPos;
  posDiff.Z(0);

  if (posDiff.Length() > this->dataPtr->pickUpRadius)
  {
    gzwarn << "Target [" << model->GetName() <<  "] too far from actor ["
           << this->dataPtr->actor->GetName() <<"]" << std::endl;
    return false;
  }

  gzmsg << "Actor [" << this->dataPtr->actor->GetName()
        << "] is following target [" << targetName << "]" << std::endl;

  this->dataPtr->target = model;
  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool FollowActorPlugin::OnUnfollow(ignition::msgs::Boolean &_res)
{
  if (!this->dataPtr->target)
  {
    _res.set_data(false);
    return false;
  }

  gzmsg << "Actor [" << this->dataPtr->actor->GetName()
        << "] stopped following target [" << this->dataPtr->target->GetName()
        << "]" << std::endl;

  this->dataPtr->target = nullptr;

  // Publish drift notification
  // 3: user requested
  ignition::msgs::UInt32 msg;
  msg.set_data(3);
  this->dataPtr->driftIgnPub.Publish(msg);

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool FollowActorPlugin::OnDriftRosService(
  ca_msgs::Drift::Request &/*_req*/,
  ca_msgs::Drift::Response &_res)
{
  _res.drift = true;
  this->dataPtr->driftFlag = true;
}
