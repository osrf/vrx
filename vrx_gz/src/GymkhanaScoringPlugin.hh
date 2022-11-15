/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef VRX_GAZEBO_GYMKHANA_SCORING_PLUGIN_HH_
#define VRX_GAZEBO_GYMKHANA_SCORING_PLUGIN_HH_

#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

#include "ScoringPlugin.hh"

namespace vrx
{
  class GymkhanaScoringPlugin : public ScoringPlugin
  {
    /// \brief Constructor.
    public: GymkhanaScoringPlugin();
  
    /// \brief Destructor.
    public: virtual ~GymkhanaScoringPlugin() override = default;
  
    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;
  
    // Documentation inherited.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;
  
    // Documentation inherited.
    private: void OnRunning() override;
 
    // Documentation inherited.
    protected: void OnFinished() override;
 
    // Documentation inherited.
    private: void ReleaseVehicle() override;
  
    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
#endif
