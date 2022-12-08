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

#ifndef VRX_WAYFINDINGSCORINGPLUGIN_HH_
#define VRX_WAYFINDINGSCORINGPLUGIN_HH_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

#include "ScoringPlugin.hh"
#include "WaypointMarkers.hh"

namespace vrx
{
/// \brief A plugin for computing the score of the wayfinding navigation task.
/// This plugin derives from the generic ScoringPlugin class. Refer to that
/// plugin for an explanation of the four states defined (Initial, Ready,
/// Running and Finished) as well as other required SDF elements.
///
/// This plugin publishes a series of poses to a topic when it enters the Ready
/// states.
///
/// In the running state it calculates a 2D pose error distance between the
/// vehicle and each of the waypoints and stores the minimum error distance
/// achieved for each waypoint so far. The current recorded minimums are
/// published to a topic as an array of doubles. The average of all minimums is
/// also updated. This value is published to a separate topic for mean error,
/// and set as the current score using the SetScore() method inherited from
/// the parent. This causes it to also appear in the task information topic.
///
/// This plugin requires the following SDF parameters:
///
/// <waypoints>: Required element specifying the set of waypoints that the
/// the vehicle should navigate through.
///
///    This block should contain at least one of these blocks:
///     <waypoint>: This block should contain a <pose> element specifying the
///     lattitude, longitude and yaw of a waypoint.
/// <markers>: Optional parameter to enable visualization markers. Check the
/// WaypointMarkers class for SDF documentation.
class WayfindingScoringPlugin : public ScoringPlugin
{
  /// \brief Constructor.
  public: WayfindingScoringPlugin();
 
  // Destructor
  public: ~WayfindingScoringPlugin() override;

  // Documentation inherited.
  public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &_eventMgr) override;

  // Documentation inherited. 
  /// \brief Callback executed at every world update.
  public: void PreUpdate(
                    const gz::sim::UpdateInfo &_info,
                    gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Private data pointer.
  GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
};

} // namespace
#endif
