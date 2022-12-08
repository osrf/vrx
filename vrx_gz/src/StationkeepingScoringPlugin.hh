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

#ifndef VRX_STATIONKEEPING_SCORING_PLUGIN_HH_
#define VRX_STATIONKEEPING_SCORING_PLUGIN_HH_

#include <gz/transport.hh>
#include "ScoringPlugin.hh"

namespace vrx
{
  /// \brief A plugin for computing the score of the station keeping task.
  /// This plugin derives from the generic ScoringPlugin class. Refer to that
  /// plugin for an explanation of the four states defined (Initial, Ready,
  /// Running and Finished) as well as other required SDF elements.
  ///
  /// This plugin publishes a goal pose to a topic when it enters the Ready
  /// state.
  ///
  /// In the running state it calculates a 2D pose error distance between the
  /// vehicle and the goal as well as a running mean error of all 2D pose errors
  /// calculated so far. The current 2D pose error is published to a topic for
  /// pose error, and the mean error is published to a task score topic. Mean
  /// error is also set as the score using the SetScore() method inherited from
  /// the parent. This causes it to also appear in the task information topic.
  ///
  /// This plugin requires the following SDF parameters:
  ///
  /// <goal_pose>: Optional parameter (vector type) specifying the latitude,
  /// longitude and yaw of the task goal. If not provided, all values default
  /// to 0.
  class StationkeepingScoringPlugin : public ScoringPlugin
  {
    /// \brief Constructor.
    public: StationkeepingScoringPlugin();

    /// \brief Destructor.
    public: ~StationkeepingScoringPlugin() override = default;

    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    /// \brief Callback executed at every world update.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
#endif
