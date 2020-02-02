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

#ifndef SERVICESIM_FOLLOWACTORPLUGIN_HH_
#define SERVICESIM_FOLLOWACTORPLUGIN_HH_

#include <memory>

#include <gazebo/common/Plugin.hh>
#include <ca_msgs/Drift.h>

namespace servicesim
{
  class FollowActorPluginPrivate;

  /// \brief Make an actor follow a target entity in the world.
  ///
  /// ## Ignition transport interface
  ///
  /// Follow service:
  ///   * Use: Ask actor to follow a (new) target
  ///   * Topic: /<namespace>/<actor_name>/follow
  ///   * Request: ignition.msgs.StringMsg with target entity name
  ///   * Response: ignition.msgs.Boolean with success / failure.
  ///               Reasons for failure:
  ///               * Target outside <pickup_radius>
  ///               * Target not found
  ///
  /// Unfollow service:
  ///   * Use: Ask actor to stop following a target, if following
  ///   * Topic: /<namespace>/<actor_name>/unfollow
  ///   * No request
  ///   * Response: ignition.msgs.Boolean with success / failure.
  ///               Reasons for failure:
  ///               * Not currently following
  ///
  /// Drift publisher:
  ///   * Use: Listen to drift notifications, i.e. when the actor stops
  ///          following the target
  ///   * Topic: /<namespace>/<actor_name>/drift
  ///   * Message: ignition.msgs.UInt32, where the number is a code for the
  ///              drift reason:
  ///              1: Target too far (beyond <max_distance>)
  ///              2: Scheduled drift time
  ///              3: User requested unfollow
  ///
  /// ## SDF parameters
  ///
  /// <namespace>: Namespace for transport
  ///
  /// <min_distance>: Distance in meters to keep from target's origin
  ///
  /// <max_distance>: Distance in meters from target's origin when to stop
  ///                 following
  ///
  /// <pickup_radius>: Distance in meters from the target's origin from which
  ///                  the /follow service will succeed
  ///
  /// <obstacle_margin>: Amount in meters by which obstacles' bounding boxes
  ///                    are expanded in all directions. The robot will stop
  ///                    before that to avoid collision
  ///
  /// <velocity>: Actor's velocity in m/s
  ///
  /// <ignore_obstacle>: Objects in the world which can be ignored for
  ///                    bounding-box collision checking
  ///
  /// ## Demo
  ///
  /// Try out the demo world: follow_actor_demo.world
  ///
  /// Request follow, for example:
  ///
  ///    ign service -s /demo/slow_follower/follow --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 1000 --req 'data: "number5"'
  ///
  /// To stop following:
  ///
  ///    ign service -s /demo/fast_follower/unfollow --reqtype ignition.msgs.Empty --reptype ignition.msgs.Boolean --timeout 1000 --req 'unused: true'
  ///
  /// Listen to drift notifications, for example:
  ///
  ///    ign topic -e -t /demo/slow_follower/drift
  class FollowActorPlugin : public gazebo::ModelPlugin
  {
    /// \brief Constructor
    public: FollowActorPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(gazebo::physics::ModelPtr _model,
        sdf::ElementPtr _sdf) override;

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
    private: void OnUpdate(const gazebo::common::UpdateInfo &_info);

    /// \brief When user requests reset.
    private: void Reset() override;

    /// \brief Checks if there is an obstacle on the way.
    /// \return True if there is
    private: bool ObstacleOnTheWay() const;

    /// \brief Callback for Ignition follow service
    /// \param[in] _req Request with target name
    /// \param[out] _res Response with true for success
    /// \param[out] _result True for success
    private: bool OnFollow(const ignition::msgs::StringMsg &_req,
        ignition::msgs::Boolean &_res);

    /// \brief Callback for Ignition unfollow service
    /// \param[out] _res Response with true for success
    /// \param[out] _result True for success
    private: bool OnUnfollow(ignition::msgs::Boolean &_res);

    /// \brief Service when the drift service is requested
    /// \param[in] _req Empty request
    /// \param[out] _res Response with true for success
    /// \param[out] _result True for success
    private: bool OnDriftRosService(
        ca_msgs::Drift::Request &_req,
        ca_msgs::Drift::Response &_res);

    /// \internal
    private: std::unique_ptr<FollowActorPluginPrivate> dataPtr;
  };
}
#endif
