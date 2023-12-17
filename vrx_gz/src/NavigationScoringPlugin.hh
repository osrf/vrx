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

#ifndef VRX_GAZEBO_NAVIGATION_SCORING_PLUGIN_HH_
#define VRX_GAZEBO_NAVIGATION_SCORING_PLUGIN_HH_

#include <gz/transport.hh>
#include "ScoringPlugin.hh"

namespace vrx
{
  /// \brief A plugin for computing the score of the navigation task.
  /// This plugin derives from the generic ScoringPlugin class. Refer to that
  /// plugin for an explanation of the four states defined (Initial, Ready,
  /// Running and Finished) as well as other required SDF elements.
  ///
  /// This plugin requires the following SDF parameters:
  ///
  /// <points_per_gate_crossed>: Number of points granted to cross any gate.
  /// <obstacle_penalty>: Specifies how many points are deducted per collision.
  /// <bonus>: Bonus granted after crossing a consecutive gate.
  /// <gates>: Specifies the collection of gates delimiting the course.
  ///
  ///   Each gate accepts the following elements:
  ///
  ///   <gate>: A gate is delimited by two markers (left and right).
  ///   The vessel should pass through the gate with the markers on the defined
  ///   right and left sides. E.g.:
  ///
  ///      <left_marker>: The name of the marker that should stay on the left
  ///      side of the vessel.
  ///      <right_marker> The name of the marker that should stay on the right
  ///      side of the vessel.
  ///
  /// Here's an example:
  /// <plugin name="vrx::NavigationScoringPlugin"
  ///         filename="libnavigationScoringPlugin.so">
  ///   <vehicle>wamv</vehicle>
  ///   <task_name>navigation_scoring_plugin</task_name>
  ///   <course_name>vrx_navigation_course</course_name>
  ///   <points_per_gate_crossed>10</points_per_gate_crossed>
  ///   <obstacle_penalty>3</obstable_penalty>
  ///   <bonus>1</bonus>
  ///   <gates>
  ///     <gate>
  ///       <left_marker>red_bound_0</left_marker>
  ///       <right_marker>green_bound_0</right_marker>
  ///     </gate>
  ///     <gate>
  ///       <left_marker>red_bound_1</left_marker>
  ///       <right_marker>green_bound_1</right_marker>
  ///     </gate>
  ///     <gate>
  ///       <left_marker>red_bound_2</left_marker>
  ///       <right_marker>green_bound_2</right_marker>
  ///     </gate>
  ///     <gate>
  ///       <left_marker>red_bound_3</left_marker>
  ///       <right_marker>green_bound_3</right_marker>
  ///     </gate>
  ///     <gate>
  ///       <left_marker>red_bound_4</left_marker>
  ///       <right_marker>green_bound_4</right_marker>
  ///     </gate>
  ///     <gate>
  ///       <left_marker>red_bound_5</left_marker>
  ///       <right_marker>green_bound_5</right_marker>
  ///     </gate>
  ///     <gate>
  ///       <left_marker>red_bound_6</left_marker>
  ///       <right_marker>green_bound_6</right_marker>
  ///     </gate>
  ///   </gates>
  /// </plugin>
  class NavigationScoringPlugin : public ScoringPlugin
  {
    /// \brief Constructor.
    public: NavigationScoringPlugin();

    /// \brief Destructor.
    public: ~NavigationScoringPlugin() override = default;

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
