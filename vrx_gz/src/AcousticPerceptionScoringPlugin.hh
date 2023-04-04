/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#ifndef VRX_ACOUSTIC_PERCEPTION_SCORING_PLUGIN_HH_
#define VRX_ACOUSTIC_PERCEPTION_SCORING_PLUGIN_HH_

#include <gz/transport.hh>
#include "ScoringPlugin.hh"

namespace vrx
{
  /// \brief A plugin for computing the score of the acoustic perception task.
  ///  This plugin derives from the generic ScoringPlugin class. Refer to that
  /// plugin for an explanation of the four states defined (Initial, Ready,
  /// Running and Finished) as well as other required SDF elements.
  ///
  /// This plugin requires the following SDF parameters:
  ///
  /// <pinger_position>: Position of the acoustic pinger.
  ///
  /// And the following SDF optional parameters:
  ///
  /// <goal_tolerance>: If the distance between the WAM-V and the pinger is
  ///   within this tolerance, we consider the task completed (meters).
  ///   Defaults to 1 meter.
  /// <markers>: Optional parameter to enable visualization markers. Check the
  /// WaypointMarkers class for SDF documentation.
  class AcousticPerceptionScoringPlugin : public ScoringPlugin
  {
    /// \brief Constructor.
    public: AcousticPerceptionScoringPlugin();

    /// \brief Destructor.
    public: ~AcousticPerceptionScoringPlugin() override = default;

    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
#endif
