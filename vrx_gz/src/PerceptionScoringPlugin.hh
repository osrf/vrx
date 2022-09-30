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
  ///
  /// <loop_forever>: Optional parameter. If true, all objects will be spawned
  /// as a circular buffer. After spawning the last element of the collection,
  /// the first one will be inserted.
  ///
  /// <frame>: Optional parameter. If present, the poses of the objects will be
  /// in the frame of this link/model. Otherwise the world frame is used.
  ///
  /// <landmark_topic>: Optional parameter.  Specify the topic to which the
  ///   plugin subscribes for receiving identification and localization msgs.
  ///   Default is "/vrx/perception/landmark"
  ///
  /// <duration>: Optional parameter. Specify the time an object sticks around.
  ///   Defaults to 5.
  ///
  /// Here's an example of a valid SDF:
  ///
  /// <plugin filename="libperception_scoring_plugin.so"
  ///         name="perception_scoring_plugin">
  ///   <vehicle>wamv</vehicle>
  ///   <task_name>perception</task_name>
  ///   <initial_state_duration>1</initial_state_duration>
  ///   <ready_state_duration>1</ready_state_duration>
  ///   <running_state_duration>300</running_state_duration>
  ///
  ///   <!-- Parameters for PopulationPlugin -->
  ///   <loop_forever>false</loop_forever>
  ///   <frame>wamv</frame>
  ///   <object_sequence>
  ///     <object>
  ///       <time>10.0</time>
  ///       <type>surmark950410</type>
  ///       <name>red_0</name>
  ///       <pose>6 0 1 0 0 0</pose>
  ///     </object>
  ///     <object>
  ///       <time>10.0</time>
  ///       <type>surmark950400</type>
  ///       <name>green_0</name>
  ///       <pose>6 6 1 0 0 0</pose>
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

    /// \brief Callback executed at every world update.
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
