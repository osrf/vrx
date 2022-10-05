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

#ifndef VRX_PERCEPTION_SCORING_PLUGIN_HH_
#define VRX_PERCEPTION_SCORING_PLUGIN_HH_

#include <memory>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

#include "ScoringPlugin.hh"

namespace vrx
{
  /// \brief A plugin that allows models to be spawned at a given location in
  /// a specific simulation time and then takes care of scoring correct
  /// identification and localization of the objects.
  ///
  /// The plugin accepts the following SDF parameters:
  ///
  /// <object_sequence>: Contains the list of objects to be populated. An object
  ///                    should be declared as an <object> element with the
  ///                    following parameters:
  ///                      <time> Simulation time to be spawned.
  ///                      <type> Model.
  ///                      <name> Landmark name.
  ///                      <pose> Initial object pose.
  ///                      <duration>: Optional parameter. Specify the time an
  ///                                  object sticks around. Defaults to 5.
  ///
  /// <frame>: Optional parameter. If present, the poses of the objects will be
  /// in the frame of this model. Otherwise the world frame is used.
  ///
  /// <landmark_topic>: Optional parameter. Specify the topic to which the
  ///   plugin subscribes for receiving identification and localization msgs.
  ///   Default is "/vrx/perception/landmark".
  ///
  /// Here's an example of a valid SDF:
  ///
  /// <plugin
  ///   filename="libPerceptionScoringPlugin.so"
  ///   name="vrx::PerceptionScoringPlugin">
  ///   <!-- Parameters for ScoringPlugin base class -->
  ///   <vehicle>wamv</vehicle>
  ///   <task_name>perception</task_name>
  ///   <initial_state_duration>10.0</initial_state_duration>
  ///   <ready_state_duration>10.0</ready_state_duration>
  ///   <running_state_duration>30</running_state_duration>
  ///   <!-- Parameters for PerceptionScoringPlugin -->
  ///   <frame>wamv</frame>
  ///   <!-- Pose of each object is expressed relative to the body frame
  ///        of the object named in the frame field - i.e., relative to
  ///        the wam-v-->
  ///   <object_sequence>
  ///     <object>
  ///       <time>5</time>
  ///       <type>mb_marker_buoy_red</type>
  ///       <name>red_0</name>
  ///       <pose>8 0 1 0 0 0</pose>
  ///     </object>
  ///     <object>
  ///       <time>5</time>
  ///       <type>mb_marker_buoy_green</type>
  ///       <name>green_0</name>
  ///       <pose>8 2 1 0 0 0</pose>
  ///     </object>
  ///     <object>
  ///       <time>15.0</time>
  ///       <type>mb_marker_buoy_black</type>
  ///       <name>black_0</name>
  ///       <pose>8 -2 1 0 0 0</pose>
  ///     </object>
  ///   </object_sequence>
  /// </plugin>
  class PerceptionScoringPlugin : public ScoringPlugin
  {
    /// \brief Constructor.
    public: PerceptionScoringPlugin();

    /// \brief Destructor.
    public: ~PerceptionScoringPlugin() override = default;

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
    private: void ReleaseVehicle() override;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
#endif
