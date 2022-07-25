/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef VRX_WAVEVISUAL_HH_
#define VRX_WAVEVISUAL_HH_

#include <memory>

#include "ignition/gazebo/System.hh"

namespace vrx
{
  // Forward declaration
  class WaveVisualPrivate;

  /// \brief A plugin for setting shaders to a visual and its params
  ///
  /// Plugin parameters:
  ///
  /// <shader>
  ///   <vertex>   Path to wave vertex program
  ///   <fragment> Path to wave fragment program
  /// <wavefield>  Wavefield parameters - see Wavefield.hh
  ///
  class WaveVisual
      : public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPreUpdate
  {
    /// \brief Constructor
    public: WaveVisual();

    /// \brief Destructor
    public: ~WaveVisual() override;

    // Documentation inherited
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<WaveVisualPrivate> dataPtr;
  };
}

#endif
