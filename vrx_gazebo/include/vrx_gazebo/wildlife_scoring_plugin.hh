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

#include <ros/ros.h>
#include <string>
#include <vector>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>
#include "vrx_gazebo/scoring_plugin.hh"

/// \brief A plugin for computing the score of the wildlife task.
/// This plugin derives from the generic ScoringPlugin class. Check out that
/// plugin for other required SDF elements.
/// This plugin uses the following SDF parameters:
///
/// * Required parameters:
/// <animals_model_name>: The top level model containing all the buoy animals.
///
/// * Optional parameters
///
/// <animals_topic>: The topic that publishes the animal poses.
///                  Defaults to "/vrx/wildlife/animals/poses"
/// <buoys>: Specifies the collection of buoys to circumnavigate, avoid, etc.
///
///   <buoy>: A buoy to circumnavigate, avoid.
///      <link_name>: The name of the main link in the buoy.
///      <goal> "avoid", "circumnavigate_clockwise" or
///             "circumnavigate_counterclockwise"
/// <engagement_distance>: At less or equal than this distance, the buoy is
///                        considered engaged. Defaults to 10 meters.
/// <obstacle_penalty>: Specifies how many seconds are added per collision.
///                     Defaults to 10 seconds.
/// <time_bonus>: Time bonus granted for each goal reached. Defaults to 30 secs.
///
/// Here's an example:
/// <plugin name="wildlife_scoring_plugin"
///         filename="libwildlife_scoring_plugin.so">
///   <!-- Common parameters -->
///   <vehicle>wamv</vehicle>
///   <task_name>wildlife_course</task_name>
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
///   <animals_model_name>animal_buoys</animals_model_name>
///   <animals_topic>/vrx/wildlife/animals/poses</animals_topic>
///   <buoys>
///     <buoy>
///       <link_name>crocodile_buoy::link</link_name>
///       <goal>avoid</goal>
///     </buoy>
///     <buoy>
///       <link_name>platypus_buoy::link</link_name>
///       <goal>circumnavigate_clockwise</goal>
///     </buoy>
///     <buoy>
///       <link_name>turtle_buoy::link</link_name>
///       <goal>circumnavigate_counterclockwise</goal>
///     </buoy>
///   </buoys>
///   <engagement_distance>10.0</engagement_distance>
///   <obstacle_penalty>10.0</obstacle_penalty>
///   <time_bonus>30.0</time_bonus>
///
/// </plugin>
class WildlifeScoringPlugin : public ScoringPlugin
{
  /// \brief All buoy goals.
  private: enum class BuoyGoal
  {
    /// \brief The goal is to stay out of the activation area.
    AVOID,

    /// \brief The goal is to circumnavigate the buoy clockwise.
    CIRCUMNAVIGATE_CLOCKWISE,

    /// \brief The goal is to circumnavigate the buoy counterclockwise
    CIRCUMNAVIGATE_COUNTERCLOCKWISE,
  };

  /// \brief All buoy states.
  private: enum class BuoyState
  {
    /// \brief Not "in" the gate and never engaged.
    NEVER_ENGAGED,

    /// \brief Not "in" the gate but was engaged at some point.
    NOT_ENGAGED,

    /// \brief Inside the area of activation of the gate.
    ENGAGED,

    /// \brief Succesfully circumnavigated.
    CIRCUMNAVIGATED,
  };

  /// \brief A buoy that is part of the wildlife task.
  private: class Buoy
  {
    /// \brief Constructor.
    /// \param[in] _buoyLink The buoy's main link.
    /// \param[in] _buoyGoal The buoy's goal.
    /// \param[in] _engagementDistance The vehicle engages with the buoy when
    ///            the distance between them is lower or equal than this value.
    public: Buoy(const gazebo::physics::LinkPtr _buoyLink,
                 const BuoyGoal _buoyGoal,
                 double _engagementDistance);

    /// \brief Update the status of this buoy.
    public: void Update();

    /// \brief Set the vehicle model.
    /// \param[in] _vehicleModel The vehicle model pointer.
    public: void SetVehicleModel(gazebo::physics::ModelPtr _vehicleModel);

    /// \brief The buoy's main link.
    public: gazebo::physics::LinkPtr link;

    /// \brief The goal.
    public: BuoyGoal goal;

    /// \brief The state of this buoy.
    public: BuoyState state = BuoyState::NEVER_ENGAGED;

    /// \brief Pointer to the vehicle that interacts with the buoy.
    public: gazebo::physics::ModelPtr vehicleModel;

    /// \brief The vehicle engages with the buoy when the distance between them
    /// is lower or equal than this value.
    public: double engagementDistance;
  };

  // Constructor.
  public: WildlifeScoringPlugin();

  // Documentation inherited.
  public: void Load(gazebo::physics::WorldPtr _world,
                    sdf::ElementPtr _sdf);

  /// \brief Parse the buoys from SDF.
  /// \param[in] _sdf The current SDF element.
  /// \return True when the buoys were successfully parsed or false otherwise.
  private: bool ParseBuoys(sdf::ElementPtr _sdf);

  /// \brief Register a new buoy.
  /// \param[in] _linkName The name of the main buoy's link.
  /// \param[in] _goal The goal associated to this buoy.
  /// \return True when the buoy has been registered or false otherwise.
  private: bool AddBuoy(const std::string &_linkName,
                        const std::string &_goal);

  /// \brief Callback executed at every world update.
  private: void Update();

  /// \brief Set the score to 0 and change the state to "finish".
  private: void Fail();

  /// \brief Publish a new ROS message with the animal locations.
  private: void PublishAnimalLocations();

  // Documentation inherited.
  private: void OnCollision() override;

  /// \brief All the buoys.
  private: std::vector<Buoy> buoys;

  /// \brief Pointer to the update event connection.
  private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief The number of WAM-V collisions.
  private: unsigned int numCollisions = 0u;

  /// \brief Number of points deducted per collision.
  private: double obstaclePenalty = 10.0;

  /// \brief The name of the topic where the animal locations are published.
  private: std::string animalsTopic = "/vrx/wildlife_animals";

  /// \brief Time bonus granted for each succcesfuly goal achieved.
  private: double timeBonus = 30.0;

  /// \brief When the vehicle is between the buoy and this distance, the vehicle
  /// engages with the buoy.
  private: double engagementDistance = 10.0;

  /// \brief The name of the model containing all the animals.
  private: std::string animalsModelName;

  /// \brief ROS node handle.
  private: std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief Publisher for the animal locations.
  private: ros::Publisher animalsPub;
};

#endif
