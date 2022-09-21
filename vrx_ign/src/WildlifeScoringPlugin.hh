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

#ifndef VRX_GAZEBO_WILDLIFE_SCORING_PLUGIN_HH_
#define VRX_GAZEBO_WILDLIFE_SCORING_PLUGIN_HH_

#include <ignition/transport.hh>
#include "ScoringPlugin.hh"

/// \brief A plugin for computing the score of the wildlife task.
/// This plugin derives from the generic ScoringPlugin class. Check out that
/// plugin for other required SDF elements.
/// This plugin uses the following SDF parameters:
///
/// * Optional parameters
///
/// <animals_topic>: The topic that publishes the animal poses.
///                  Defaults to "/vrx/wildlife/animals/poses"
/// <buoys>: Specifies the collection of buoys to circumnavigate, avoid, etc.
///
///   <buoy>: A buoy to circumnavigate, avoid.
///      <model_name>: The name of the model representing the buoy
///      <link_name>: The name of the main link in the model.
///      <goal> "avoid", "circumnavigate_clockwise" or
///             "circumnavigate_counterclockwise"
/// <engagement_distance>: At less or equal than this distance, the buoy is
///                        considered engaged. Defaults to 10 meters.
/// <time_bonus>: Time bonus granted for each goal reached. Defaults to 30 secs.
///
/// Here's an example:
/// <plugin name="wildlife_scoring_plugin"
///         filename="libwildlife_scoring_plugin.so">
///   <!-- Common parameters -->
///   <vehicle>wamv</vehicle>
///   <task_name>wildlife</task_name>
///   <initial_state_duration>10</initial_state_duration>
///   <ready_state_duration>10</ready_state_duration>
///   <running_state_duration>300</running_state_duration>
///   <collision_buffer>10</collision_buffer>
///   <release_joints>
///     <joint>
///       <name>wamv_external_pivot_joint</name>
///     </joint>
///     <joint>
///       <name>wamv_external_riser</name>
///     </joint>
///   </release_joints>
///
///   <!-- wildlife specific parameters -->
///   <animals_topic>/vrx/wildlife/animals/poses</animals_topic>
///   <buoys>
///     <buoy>
///       <model_name>crocodile_buoy</model_name>
///       <link_name>link</link_name>
///       <goal>avoid</goal>
///     </buoy>
///     <buoy>
///       <model_name>platypus_buoy</model_name>
///       <link_name>link</link_name>
///       <goal>circumnavigate_clockwise</goal>
///     </buoy>
///     <buoy>
///       <model_name>turtle_buoy</model_name>
///       <link_name>link</link_name>
///       <goal>circumnavigate_counterclockwise</goal>
///     </buoy>
///   </buoys>
///   <engagement_distance>10.0</engagement_distance>
///   <time_bonus>30.0</time_bonus>
/// </plugin>

namespace vrx
{
    class WildlifeScoringPlugin : public ScoringPlugin
    {
    protected: class VirtualGate;

    protected: class Buoy;

    protected: enum class GateState;

    protected: enum class BuoyState;

    protected: enum class BuoyGoal;

        /// \brief Constructor.
    public:
        WildlifeScoringPlugin();

        /// \brief Destructor.
    public:
        ~WildlifeScoringPlugin() override = default;

        // Documentation inherited.
    public:
        void Configure(const ignition::gazebo::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       ignition::gazebo::EntityComponentManager &_ecm,
                       ignition::gazebo::EventManager &_eventMgr) override;

        /// \brief Callback executed at every world update.
    public:
        void PreUpdate(
            const ignition::gazebo::UpdateInfo &_info,
            ignition::gazebo::EntityComponentManager &_ecm) override;

        // Documentation inherited.
    protected:
        void OnReady() override;

        // Documentation inherited.
    protected:
        void OnRunning() override;

        // Documentation inherited.
    protected:
        void OnFinished() override;

        // Documentation
    protected:
        void OnCollision() override;

        /// \brief Private data pointer.
        IGN_UTILS_UNIQUE_IMPL_PTR(dataPtr)
    };
}



#endif
