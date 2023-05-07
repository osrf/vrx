/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef VRX_WILDLIFE_SCORING_PLUGIN_HH_
#define VRX_WILDLIFE_SCORING_PLUGIN_HH_

#include <memory>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

#include "ScoringPlugin.hh"

namespace vrx
{
  /// \brief A plugin for computing the score of the wildlife task.
  /// This plugin derives from the generic ScoringPlugin class. Check out that
  /// plugin for other required SDF elements.
  /// This plugin uses the following SDF parameters:
  ///
  /// * Optional parameters
  ///
  /// <topic_prefix>: The topic prefix used to publish an animal pose.
  ///                 Defaults to "/vrx/wildlife/animal". The final topic will
  ///                 be constructed by appending a numeric index
  ///                 starting from 0, followed by "/pose".
  ///                 E.g. of final topic: "/vrx/wildfile/animal0/pose"
  ///                 Note that this topic will be advertised for each animal.
  ///
  /// <publication_frequency>: How many messages per second are published with
  ///                          the animal poses (Hz). Defaults to 1 Hz.
  ///
  /// <buoys>: Specifies the collection of buoys to circumnavigate, avoid, etc.
  ///
  ///   <buoy>: A buoy to circumnavigate, avoid.
  ///      <name>: The scoped name of the model representing the buoy
  ///      <goal> "avoid", "circumnavigate_clockwise" or
  ///             "circumnavigate_counterclockwise"
  /// <engagement_distance>: At less or equal than this distance, the buoy is
  ///                        considered engaged. Defaults to 10 meters.
  /// <time_bonus>: Time bonus granted for each goal reached.
  ///               Defaults to 30 secs.
  ///
  /// Here's an example:
  /// <plugin
  ///   filename="libWildlifeScoringPlugin.so"
  ///   name="vrx::WildlifeScoringPlugin">
  ///
  ///   <!-- Common parameters -->
  ///   <vehicle>wamv</vehicle>
  ///   <task_name>wildlife</task_name>
  ///   <initial_state_duration>10</initial_state_duration>
  ///   <ready_state_duration>10</ready_state_duration>
  ///   <running_state_duration>300</running_state_duration>
  ///   <collision_buffer>10</collision_buffer>
  ///
  ///   <!-- wildlife specific parameters -->
  ///   <buoys>
  ///     <buoy>
  ///       <name>crocodile::link</name>
  ///       <goal>avoid</goal>
  ///     </buoy>
  ///     <buoy>
  ///       <name>platypus::link</name>
  ///       <goal>circumnavigate_clockwise</goal>
  ///     </buoy>
  ///     <buoy>
  ///       <name>turtle::link</name>
  ///       <goal>circumnavigate_counterclockwise</goal>
  ///     </buoy>
  ///   </buoys>
  ///   <engagement_distance>10.0</engagement_distance>
  ///   <time_bonus>30.0</time_bonus>
  ///   <topic_prefix>/vrx/wildlife/animal</topic_prefix>
  /// <plugin>
  class WildlifeScoringPlugin : public ScoringPlugin
  {
    /// \brief Constructor.
    public: WildlifeScoringPlugin();

    /// \brief Destructor.
    public: ~WildlifeScoringPlugin() override = default;

    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited.
    private: void OnCollision() override;

    // Documentation inherited.
    private: virtual void OnFinished() override;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
#endif
