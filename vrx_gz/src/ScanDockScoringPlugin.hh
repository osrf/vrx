/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef VRX_SCANDOCK_SCORING_PLUGIN_HH_
#define VRX_SCANDOCK_SCORING_PLUGIN_HH_

#include <memory>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

#include "ScoringPlugin.hh"

namespace vrx
{
  /// \brief A plugin for computing the score of the scan, dock, deliver task.
  /// This plugin derives from the generic ScoringPlugin class. Check out that
  /// plugin for other required SDF elements.
  /// This plugin requires the following SDF parameters:
  ///
  /// <enable_color_checker>: Optional parameter to turn off color checker
  /// service. Default is true.
  /// <color_sequence_topic>: Optional parameter with the topic used to
  /// receive the color submission. Default is "/vrx/scan_dock/color_sequence".
  /// <color_1>: Expected 1st color of the sequence (RED, GREEN, BLUE, YELLOW).
  /// <color_2>: Expected 2nd color of the sequence (RED, GREEN, BLUE, YELLOW).
  /// <color_3>: Expected 3rd color of the sequence (RED, GREEN, BLUE, YELLOW).
  /// <color_bonus_points>: Points granted when the color sequence is correct.
  /// Default value is 10.
  /// Here's an example:
  /// <plugin name="vrx::ScanDockScoringPlugin"
  ///               filename="libScanDockScoringPlugin.so">
  ///   <!-- Parameters for scoring_plugin -->
  ///   <vehicle>wamv</vehicle>
  ///   <task_name>scan_dock_deliver</task_name>
  ///   <initial_state_duration>3</initial_state_duration>
  ///   <ready_state_duration>3</ready_state_duration>
  ///   <running_state_duration>300</running_state_duration>
  ///   <release_topic>/vrx/release</release_topic>
  ///
  ///   <!-- Color sequence checker -->
  ///   <color_sequence_topic>/vrx/scan_dock_deliver/color_sequence</color_sequence_topic>
  ///   <color_1>red</color_1>
  ///   <color_2>green</color_2>
  ///   <color_3>blue</color_3>
  /// </plugin>
  class ScanDockScoringPlugin : public ScoringPlugin
  {
    /// \brief Constructor.
    public: ScanDockScoringPlugin();

    /// \brief Destructor.
    public: ~ScanDockScoringPlugin() override = default;

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

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
#endif
