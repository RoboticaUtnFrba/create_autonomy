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
#include <ignition/math/Vector3.hh>

#include <gazebo/common/Animation.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/KeyFrame.hh>
#include <gazebo/physics/physics.hh>

#include "TrajectoryActorPlugin.hh"

using namespace gazebo;
using namespace servicesim;
GZ_REGISTER_MODEL_PLUGIN(servicesim::TrajectoryActorPlugin)

class servicesim::TrajectoryActorPluginPrivate
{
  /// \brief Pointer to the actor.
  public: physics::ActorPtr actor{nullptr};

  /// \brief Velocity of the actor
  public: double velocity{0.8};

  /// \brief List of connections such as WorldUpdateBegin
  public: std::vector<event::ConnectionPtr> connections;

  /// \brief List of targets
  public: std::vector<ignition::math::Pose3d> targets;

  /// \brief Current target index
  public: unsigned int currentTarget{0};

  /// \brief Radius in meters around target pose where we consider it was
  /// reached.
  public: double targetRadius{0.5};

  /// \brief Margin by which to increase an obstacle's bounding box on every
  /// direction (2x per axis).
  public: double obstacleMargin{0.5};

  /// \brief Time scaling factor. Used to coordinate translational motion
  /// with the actor's walking animation.
  public: double animationFactor{5.1};

  /// \brief Time of the last update.
  public: common::Time lastUpdate;

  /// \brief Time when the corner starts
  public: common::Time firstCornerUpdate;

  /// \brief List of models to avoid
  public: std::vector<std::string> obstacles;

  /// \brief Animation for corners
  public: common::PoseAnimation *cornerAnimation{nullptr};

  /// \brief Frequency in Hz to update
  public: double updateFreq{60};
};

/////////////////////////////////////////////////
TrajectoryActorPlugin::TrajectoryActorPlugin()
    : dataPtr(new TrajectoryActorPluginPrivate)
{
}

/////////////////////////////////////////////////
void TrajectoryActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);

  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&TrajectoryActorPlugin::OnUpdate, this, std::placeholders::_1)));

  // Update frequency
  if (_sdf->HasElement("update_frequency"))
    this->dataPtr->updateFreq = _sdf->Get<double>("update_frequency");

  // Read in the velocity
  if (_sdf->HasElement("velocity"))
    this->dataPtr->velocity = _sdf->Get<double>("velocity");

  // Read in the target poses
  auto targetElem = _sdf->GetElement("target");
  while (targetElem)
  {
    this->dataPtr->targets.push_back(targetElem->Get<ignition::math::Pose3d>());
    targetElem = targetElem->GetNextElement("target");
  }

  // Read in the target mradius
  if (_sdf->HasElement("target_radius"))
    this->dataPtr->targetRadius = _sdf->Get<double>("target_radius");

  // Read in the obstacle margin
  if (_sdf->HasElement("obstacle_margin"))
    this->dataPtr->obstacleMargin = _sdf->Get<double>("obstacle_margin");

  // Read in the animation factor
  if (_sdf->HasElement("animation_factor"))
    this->dataPtr->animationFactor = _sdf->Get<double>("animation_factor");

  // Read in the obstacles
  if (_sdf->HasElement("obstacle"))
  {
    auto obstacleElem = _sdf->GetElement("obstacle");
    while (obstacleElem)
    {
      auto name = obstacleElem->Get<std::string>();
      this->dataPtr->obstacles.push_back(name);
      obstacleElem = obstacleElem->GetNextElement("obstacle");
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
    gazebo::physics::TrajectoryInfoPtr trajectoryInfo(new physics::TrajectoryInfo());
    trajectoryInfo->type = animation;
    trajectoryInfo->duration = 1.0;

    this->dataPtr->actor->SetCustomTrajectory(trajectoryInfo);
  }
}

/////////////////////////////////////////////////
void TrajectoryActorPlugin::Reset()
{
  this->dataPtr->currentTarget = 0;
  this->dataPtr->cornerAnimation = nullptr;
  this->dataPtr->lastUpdate = common::Time::Zero;
}

/////////////////////////////////////////////////
bool TrajectoryActorPlugin::ObstacleOnTheWay() const
{
  auto actorWorld = ignition::math::Matrix4d(this->dataPtr->actor->WorldPose());
  auto world = this->dataPtr->actor->GetWorld();

  // Iterate over all models in the world
  for (unsigned int i = 0; i < world->ModelCount(); ++i)
  {
    // Skip if it's not an obstacle
    // Fixme: automatically adding all actors to obstacles
    auto model = world->ModelByIndex(i);
    auto act = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);
    if (!act &&
        std::find(this->dataPtr->obstacles.begin(),
                  this->dataPtr->obstacles.end(), model->GetName()) ==
                  this->dataPtr->obstacles.end())
    {
      continue;
    }
    if (act && act == this->dataPtr->actor)
      continue;

    auto modelWorld = ignition::math::Matrix4d(model->WorldPose());

    // Model in actor's frame
    auto modelActor = actorWorld.Inverse() * modelWorld;

    auto modelPos = ignition::math::Vector3d(
        modelActor(0, 3),
        modelActor(1, 3),
        modelActor(2, 3));

    // Check not only if near, but also if in front of the actor
    if (modelPos.Length() < this->dataPtr->obstacleMargin &&
        std::abs(modelActor(0, 3)) < this->dataPtr->obstacleMargin * 0.4 &&
        modelActor(2, 3) > 0)
    {
      return true;
    }

    // TODO: Improve obstacle avoidance. Some ideas: check contacts, ray-query
    // the path forward, check against bounding box of each collision shape...

    // Note: Used to check against model bounding box, but that was unreliable
    // for URDF and actors (our 2 use cases :))
  }
  return false;
}

/////////////////////////////////////////////////
void TrajectoryActorPlugin::UpdateTarget()
{
  // Current actor position
  auto actorPos = this->dataPtr->actor->WorldPose().Pos();

  // Current target
  auto target = this->dataPtr->targets[this->dataPtr->currentTarget].Pos();

  // 2D distance to target
  auto posDiff = target - actorPos;
  posDiff.Z(0);

  double distance = posDiff.Length();

  // Still far from target?
  if (distance > this->dataPtr->targetRadius)
    return;

  // Move on to next target
  this->dataPtr->currentTarget++;
  if (this->dataPtr->currentTarget > this->dataPtr->targets.size() - 1)
    this->dataPtr->currentTarget = 0;
}

/////////////////////////////////////////////////
void TrajectoryActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->dataPtr->lastUpdate).Double();

  if (dt < 1/this->dataPtr->updateFreq)
    return;

  this->dataPtr->lastUpdate = _info.simTime;

  // Don't move if there's an obstacle on the way
  if (this->ObstacleOnTheWay())
    return;

  // Target
  this->UpdateTarget();

  // Current pose - actor is oriented Y-up and Z-front
  auto actorPose = this->dataPtr->actor->WorldPose();

  // Current target
  auto targetPose = this->dataPtr->targets[this->dataPtr->currentTarget];

  // Direction to target
  auto dir = (targetPose.Pos() - actorPose.Pos()).Normalize();

  // TODO: generalize for actors facing other directions
  auto currentYaw = actorPose.Rot().Yaw();

  // Difference to target
  ignition::math::Angle yawDiff = atan2(dir.Y(), dir.X()) + IGN_PI_2 - currentYaw;
  yawDiff.Normalize();

  // Rotate if needed
  if (std::abs(yawDiff.Radian()) > IGN_DTOR(10))
  {
    // Not rotating yet
    if (!this->dataPtr->cornerAnimation)
    {
      // Previous target (we assume we just reached it)
      int previousTarget = this->dataPtr->currentTarget - 1;
      if (previousTarget < 0)
        previousTarget = this->dataPtr->targets.size() - 1;

      auto prevTargetPos = this->dataPtr->targets[previousTarget].Pos();

      // Direction from previous target to current target
      auto prevDir = (targetPose.Pos() - prevTargetPos).Normalize() *
          this->dataPtr->targetRadius;

      // Curve end point
      auto endPt = prevTargetPos + prevDir;

      // Total time to finish the curve, we try to keep about the same speed,
      // it will be a bit slower because we're doing an arch, not a straight
      // line
      auto curveDist = (endPt - actorPose.Pos()).Length();
      auto curveTime = curveDist / this->dataPtr->velocity;

      // Use pose animation for spline
      this->dataPtr->cornerAnimation = new common::PoseAnimation("anim", curveTime, false);

      // Start from actor's current pose
      auto start = this->dataPtr->cornerAnimation->CreateKeyFrame(0.0);
      start->Translation(actorPose.Pos());
      start->Rotation(actorPose.Rot());

      // End of curve
      auto endYaw = atan2(prevDir.Y(), prevDir.X()) + IGN_PI_2;
      auto end = this->dataPtr->cornerAnimation->CreateKeyFrame(curveTime);
      end->Translation(endPt);
      end->Rotation(ignition::math::Quaterniond(IGN_PI_2, 0, endYaw));

      this->dataPtr->firstCornerUpdate = _info.simTime;
    }

    // Get point in curve
    auto cornerDt = (_info.simTime - this->dataPtr->firstCornerUpdate).Double();
    common::PoseKeyFrame pose(cornerDt);
    this->dataPtr->cornerAnimation->SetTime(cornerDt);
    this->dataPtr->cornerAnimation->GetInterpolatedKeyFrame(pose);

    actorPose.Pos() = pose.Translation();
    actorPose.Rot() = ignition::math::Quaterniond(IGN_PI_2, 0, pose.Rotation().Yaw());
  }
  else
  {
    this->dataPtr->cornerAnimation = nullptr;

    actorPose.Pos() += dir * this->dataPtr->velocity * dt;

    // TODO: remove hardcoded roll
    actorPose.Rot() = ignition::math::Quaterniond(IGN_PI_2, 0, currentYaw + yawDiff.Radian());
  }

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (actorPose.Pos() -
      this->dataPtr->actor->WorldPose().Pos()).Length();

  // Update actor
  this->dataPtr->actor->SetWorldPose(actorPose, false, false);
  this->dataPtr->actor->SetScriptTime(this->dataPtr->actor->ScriptTime() +
    (distanceTraveled * this->dataPtr->animationFactor));
}
