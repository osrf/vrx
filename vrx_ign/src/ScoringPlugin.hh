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

#ifndef VRX_ScoringPlugin_HH_
#define VRX_ScoringPlugin_HH_

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

namespace vrx
{
  /// \brief A plugin that provides common functionality to any scoring plugin.
  /// This plugin defines four different task states:
  ///
  /// * Initial: The vehicle might be locked to the world via some joints with
  /// different constraints. This state is used to stabilize the vehicle and let
  /// it settle for a while.
  /// * Ready: If the vehicle was locked, it will be released in this state. The
  /// task hasn't started yet, but the vehicle can be controlled and prepared for
  /// the start of the task.
  /// * Running: The task has started. The clockwatch task starts too.
  /// * Finished: The maximum allowed task time has been reached (time out) or
  /// the task has been completed. Other plugins derived from the ScoringPlugin
  /// class can call the Finish() method to trigger the completion of the task.
  ///
  /// The plugin also exposes a pair of methods [Set]Score() for setting and
  /// getting a score.
  ///
  /// Derived plugins can also override the OnReady(), OnRunning(),
  /// and OnFinished() to be notified when the task transitions into the "ready".
  /// "running" or "finished" states respectively.
  ///
  /// The plugin publishes task information on a ROS topic at 1Hz.
  ///
  /// This plugin uses the following SDF parameters:
  ///
  /// <vehicle>: Required parameter (string type) with the name of the main
  /// vehicle to be under control during the task.
  ///
  /// <task_name>: Required parameter specifying the task name (string type).
  ///
  /// <task_info_topic>: Optional parameter (string type)
  /// containing the ROS topic name to publish the task stats. The default
  /// topic name is /vrx/task/info .
  ///
  /// <contact_debug_topic>: Optional parameter (string type)
  /// containing the ROS topic name to
  /// publish every instant a collision with the wamv is happening.
  /// Default is /vrx/debug/contact.
  ///
  /// <per_plugin_exit_on_completion>: Specifies whether to shut down after
  /// completion, for this specific plugin.
  /// Different from environment variable VRX_EXIT_ON_COMPLETION, which is for
  /// every plugin in the current shell. Environment variable overwrites this
  /// parameter.
  ///
  /// <initial_state_duration>: Optional parameter (double type) specifying the
  /// amount of seconds that the plugin will be in the "initial" state.
  ///
  /// <ready_state_duration>: Optional parameter (double type) specifying the
  /// amount of seconds that the plugin will be in the "ready" state.
  ///
  /// <running_state_duration>: Optional parameter (double type) specifying the
  /// amount of maximum seconds that the plugin will be in the "running" state.
  /// Note that this parameter specifies the maximum task time.
  ///
  /// <collision_buffer>: Optional parameter (double type) specifying the
  /// minimum amount of seconds between two collisions. If N collisions happen
  /// within this time frame, only one will be counted.
  ///
  /// <release_joints>: Optional element specifying the collection of joints that
  /// should be dettached when transitioning to the "ready" state.
  ///
  ///   This block should contain at least one of these blocks:
  ///   <joint>: This block should contain a <name> element with the name of the
  ///   joint to release.
  ///
  /// <silent>: Optional element (boolean type) specifying if we want to
  /// avoid showing the state messages.
  ///
  /// Here's an example:
  /// <plugin name="scoring_plugin"
  ///         filename="libscoring_plugin.so">
  ///   <vehicle>wamv</vehicle>
  ///   <task_name>navigation_course</task_name>
  ///   <initial_state_duration>10</initial_state_duration>
  ///   <ready_state_duration>10</ready_state_duration>
  ///   <running_state_duration>30</running_state_duration>
  ///   <release_joints>
  ///     <joint>
  ///       <name>wamv_external_pivot_joint</name>
  ///     </joint>
  ///     <joint>
  ///       <name>wamv_external_riser</name>
  ///     </joint>
  ///   </release_joints>
  /// </plugin>
  class ScoringPlugin
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
  {
    /// \brief Constructor.
    public: ScoringPlugin();

    /// \brief Destructor.
    public: ~ScoringPlugin() override = default;

    // Documentation inherited.
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    IGN_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}

#endif
