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

#ifndef SERVICESIM_COLLISIONACTORPLUGIN_HH_
#define SERVICESIM_COLLISIONACTORPLUGIN_HH_

#include <gazebo/common/Plugin.hh>

namespace servicesim
{
  /// \brief This plugin enables collisions on an Actor, and
  /// optionally applies a scaling factor and pose offset to each of the
  /// Actor's box collisions.
  /// Collisions only work on enabled objects. ODE has a auto-disable
  /// feature that "disables" an object when it comes to rest. An actor will
  /// pass through these objects.
  class CollisionActorPlugin : public gazebo::ModelPlugin
  {
    /// \brief Constructor
    public: CollisionActorPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(gazebo::physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);
  };
}
#endif
