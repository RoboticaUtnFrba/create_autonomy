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

#ifndef SERVICESIM_PLUGINS_WANDERINGACTORPLUGIN_HH_
#define SERVICESIM_PLUGINS_WANDERINGACTORPLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include "gazebo/util/system.hh"

namespace servicesim
{
  class TrajectoryActorPluginPrivate;

  class GAZEBO_VISIBLE TrajectoryActorPlugin : public gazebo::ModelPlugin
  {
    /// \brief Constructor
    public: TrajectoryActorPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
        override;

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
    private: void OnUpdate(const gazebo::common::UpdateInfo &_info);

    /// \brief When user requests reset.
    private: void Reset() override;

    /// \brief Checks if there is an obstacle on the way.
    /// \return True if there is
    private: bool ObstacleOnTheWay() const;

    /// \brief Update target
    private: void UpdateTarget();

    /// \internal
    private: TrajectoryActorPluginPrivate *dataPtr;
  };
}
#endif
