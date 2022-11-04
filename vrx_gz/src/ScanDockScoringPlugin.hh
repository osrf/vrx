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
  /// service - default is true.
  /// <robot_namespace>: Optional parameter with the ROS namespace.
  /// <color_sequence_service>: Optional paramter with the ROS service used to
  /// receive the color submission.
  /// <color_topic>: Optional gazebo topic used to publish the color sequence
  ///   defaults to /vrx/light_buoy/new_pattern
  /// <color_1>: Expected first color of the sequence (RED, GREEN, BLUE, YELLOW).
  /// <color_2>: Expected second color of the sequence (RED, GREEN, BLUE, YELLOW).
  /// <color_3>: Expected third color of the sequence (RED, GREEN, BLUE, YELLOW).
  /// <color_bonus_points>: Points granted when the color sequence is correct.
  /// Default value is 10.
  /// <bays>: Contains at least one of the following blocks:
  ///   <bay>: A bay represents a potential play where a vehicle can dock. It has
  ///   the following required elements:
  ///     <name>The name of the bay. This is used for debugging only.
  ///     <internal_activation_topic>The gazebo topic used to receive
  ///     notifications from the internal activation zone.
  ///     <external_activation_topic>The gazebo topic used to receive
  ///     notifications from the external activation zone.
  ///     <min_dock_time>Minimum amount of seconds to stay docked to be
  ///     considered a fully successfull dock.
  ///     <dockAllowed> Whether is allowed to dock in this bay or not.
  /// <dock_bonus_points>: Points granted when the vehicle successfully
  /// dock-and-undock in any bay.
  /// Default value is 10.
  /// <correct_dock_bonus_points>: Points granted when the vehicle successfully
  /// dock-and-undock in the specified bay.
  /// Default value is 10.
  /// <symbol>: Required string with format <COLOR>_<SHAPE>, where
  /// color can be "red", "green", "blue", "yellow" and color can be "triangle",
  /// "circle", "cross", "rectangle". If this parameter is present, a ROS message
  /// will be sent in OnReady(). The vehicle should dock in the bay matching this
  /// color and shape.
  /// <targets>: Contains at least one of the following blocks:
  ///   <target>: A shooting target. It has the following elements:
  ///     <topic>: The name of the topic where the "libContainPlugin" publishes
  ///              information for this target.
  ///     <bonus_points>: The points awarded for hitting this target.
  ///
  /// Here's an example:
  /// <plugin name="scan_dock_scoring_plugin"
  ///               filename="libscan_dock_scoring_plugin.so">
  ///   <!-- Parameters for scoring_plugin -->
  ///   <vehicle>wamv</vehicle>
  ///   <task_name>scan_dock_deliver</task_name>
  ///   <initial_state_duration>3</initial_state_duration>
  ///   <ready_state_duration>3</ready_state_duration>
  ///   <running_state_duration>300</running_state_duration>
  ///   <release_joints>
  ///     <joint>
  ///       <name>wamv_external_pivot_joint</name>
  ///     </joint>
  ///     <joint>
  ///       <name>wamv_external_riser</name>
  ///     </joint>
  ///   </release_joints>
  ///
  ///   <!-- Color sequence checker -->
  ///   <robot_namespace>vrx</robot_namespace>
  ///   <color_sequence_service>scan_dock/color_sequence</color_sequence_service>
  ///   <color_1>blue</color_1>
  ///   <color_2>green</color_2>
  ///   <color_3>red</color_3>
  ///
  ///   <!-- Dock checkers -->
  ///   <bays>
  ///     <bay>
  ///       <name>bay1</name>
  ///       <internal_activation_topic>/vrx/dock_2022/bay_1/contain
  ///       </internal_activation_topic>
  ///       <external_activation_topic>/vrx/dock_2022/bay_1_external/contain
  ///       </external_activation_topic>
  ///       <symbol_topic>/vrx/dock_2022_placard1/symbol</symbol_topic>
  ///       <min_dock_time>10.0</min_dock_time>
  ///       <dock_allowed>false</dock_allowed>
  ///       <symbol>red_circle</symbol>
  ///     </bay>

  ///     <bay>
  ///       <name>bay2</name>
  ///       <internal_activation_topic>/vrx/dock_2022/bay_2/contain
  ///       </internal_activation_topic>
  ///       <external_activation_topic>/vrx/dock_2022/bay_2_external/contain
  ///       </external_activation_topic>
  ///       <symbol_topic>/vrx/dock_2022_placard2/symbol</symbol_topic>
  ///       <min_dock_time>10.0</min_dock_time>
  ///       <dock_allowed>true</dock_allowed>
  ///       <symbol>blue_circle</symbol>
  ///     </bay>

  ///     <bay>
  ///       <name>bay3</name>
  ///       <internal_activation_topic>/vrx/dock_2022/bay_3/contain
  ///       </internal_activation_topic>
  ///       <external_activation_topic>/vrx/dock_2022/bay_3_external/contain
  ///       </external_activation_topic>
  ///       <symbol_topic>/vrx/dock_2022_placard3/symbol</symbol_topic>
  ///       <min_dock_time>10.0</min_dock_time>
  ///       <dock_allowed>true</dock_allowed>
  ///       <symbol>yellow_rectangle</symbol>
  ///     </bay>
  ///   </bays>
  ///
  ///   <!-- Shooting targets -->
  ///   <targets>
  ///     <!-- Placard #1 -->
  ///     <target>
  ///       <topic>vrx/dock_2022_placard1_big_hole/contain</topic>
  ///       <bonus_points>5</bonus_points>
  ///     </target>
  ///     <target>
  ///       <topic>vrx/dock_2022_placard1_small_hole/contain</topic>
  ///       <bonus_points>10</bonus_points>
  ///     </target>

  ///     <!-- Placard #2 -->
  ///     <target>
  ///       <topic>vrx/dock_2022_placard2_big_hole/contain</topic>
  ///       <bonus_points>5</bonus_points>
  ///     </target>
  ///     <target>
  ///       <topic>vrx/dock_2022_placard2_small_hole/contain</topic>
  ///       <bonus_points>10</bonus_points>
  ///     </target>

  ///     <!-- Placard #3 -->
  ///     <target>
  ///       <topic>vrx/dock_2022_placard3_big_hole/contain</topic>
  ///       <bonus_points>5</bonus_points>
  ///     </target>
  ///     <target>
  ///       <topic>vrx/dock_2022_placard3_small_hole/contain</topic>
  ///       <bonus_points>10</bonus_points>
  ///     </target>
  ///   </targets>
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
    private: void OnReady() override;

    // Documentation inherited.
    private: void OnRunning() override;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
#endif
