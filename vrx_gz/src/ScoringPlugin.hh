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

#ifndef VRX_SCORINGPLUGIN_HH_
#define VRX_SCORINGPLUGIN_HH_

#include <gz/msgs/contacts.pb.h>
#include <chrono>
#include <memory>
#include <string>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
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
  /// <plugin
  ///   filename="libScoringPlugin.so"
  ///   name="vrx::ScoringPlugin">
  ///   <vehicle>wamv</vehicle>
  ///   <task_name>my_task</task_name>
  ///   <task_info_topic>/vrx/task/info</task_info_topic>
  ///   <initial_state_duration>10</initial_state_duration>
  ///   <ready_state_duration>10</ready_state_duration>
  ///   <running_state_duration>300</running_state_duration>
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
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor.
    public: ScoringPlugin();

    /// \brief Destructor.
    public: ~ScoringPlugin() override = default;

    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Get the current score.
    /// \return The current score.
    protected: double Score() const;

    /// \brief Set the score.
    /// \param[in] _newScore The new score.
    protected: void SetScore(double _newScore);

    /// \brief Get the task name.
    /// \return Task name.
    protected: std::string TaskName() const;

    /// \brief Get the task state.
    /// \return Task state.
    protected: std::string TaskState() const;

    /// \brief Get running duration.
    /// \return the The running state duration in seconds.
    protected: double RunningStateDuration() const;

    /// \brief Elapsed time in the running state.
    /// \return The elapsed time in the running state.
    protected: std::chrono::duration<double> ElapsedTime() const;

    /// \brief Remaining time in the running state.
    /// \return The remaining time in the running state.
    protected: std::chrono::duration<double> RemainingTime() const;

    /// \brief Set the score in case of timeout.
    /// \brief[in] _timeoutScore The timeout score.
    protected: void SetTimeoutScore(double _timeoutScore);

    /// \brief Get the timeoutScore.
    /// \return The timeout score.
    protected: double TimeoutScore() const;

    /// \brief Finish the current task. Note that the simulation termination
    /// (if happening) will be delayed <exit_delay> seconds.
    /// This will set the "finished" flag in the task message to true.
    protected: void Finish();

    /// \brief Get the vehicleName.
    /// \return The vehicle name.
    protected: std::string VehicleName() const;

    /// \brief Get the vehicle entity.
    /// \return The vehicle entity.
    protected: gz::sim::Entity VehicleEntity() const;

    /// \brief Get the number of WAM-V collisions.
    /// \return Number of collisions
    protected: uint16_t NumCollisions() const;

    /// \brief Callback executed when the task state transition into "ready".
    protected: virtual void OnReady();

    /// \brief Callback executed when the task state transition into "running".
    protected: virtual void OnRunning();

    /// \brief Callback executed when the task state transition into "finished".
    protected: virtual void OnFinished();

    /// \brief Callback executed when a collision is detected in the vehicle.
    protected: virtual void OnCollision();

    /// \brief Release the vehicle in case it's locked.
    protected: virtual void ReleaseVehicle();

    /// \brief Update the state of the current task.
    private: void UpdateTaskState();

    /// \brief Callback used to receive contacts information.
    /// \param[in] _contacts The message containing contact information.
    private: void OnContacts(const gz::msgs::Contacts &_contacts);

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}

#endif
