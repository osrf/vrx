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
#include <memory>
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

  /// \brief All gate states.
  private: enum class GateState
  {
    /// \brief Not "in" the gate.
    VEHICLE_OUTSIDE,

    /// \brief Before the gate.
    VEHICLE_BEFORE,

    /// \brief After the gate.
    VEHICLE_AFTER,

    /// \brief Gate crossed!
    CROSSED,
  };

  /// \brief A virtual gate to help detecting circumnavigation.
  private: class VirtualGate
  {
    /// \brief Constructor.
    /// \param[in] _leftMakerLink The link of the buoy.
    /// \param[in] _offset The offset from the buoy that delimitates the gate.
    /// \param[in] _width The width of the gate.
    public: VirtualGate(const gazebo::physics::LinkPtr _leftMakerLink,
                        const ignition::math::Vector3d &_offset,
                        double _width);

    /// \brief Where is the given robot pose with respect to the gate?
    /// \param _robotWorldPose Pose of the robot, in the world frame.
    /// \return The gate state given the current robot pose.
    public: GateState IsPoseInGate(
      const ignition::math::Pose3d &_robotWorldPose) const;

    /// \brief Recalculate the pose of the gate.
    public: void Update();

    /// \brief The left marker (buoy).
    public: const gazebo::physics::LinkPtr leftMarkerLink;

    /// \brief The offset of the right marker with respect to the left marker
    /// in world coordinates.
    public: const ignition::math::Vector3d offset;

    /// \brief The width of the gate in meters.
    public: const double width;

    /// \brief The center of the gate in the world frame. Note that the roll and
    /// pitch are ignored. Only yaw is relevant and it points into the direction
    /// in which the gate should be crossed.
    public: ignition::math::Pose3d pose;

    /// \brief The state of this gate.
    public: GateState state = GateState::VEHICLE_OUTSIDE;
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

    /// \brief The number of virtual gates;
    private: const unsigned int kNumVirtualGates = 8u;

    /// \brief The buoy's main link.
    public: const gazebo::physics::LinkPtr link;

    /// \brief The goal.
    public: const BuoyGoal goal;

    /// \brief The vehicle engages with the buoy when the distance between them
    /// is lower or equal than this value.
    public: const double engagementDistance;

    /// \brief The state of this buoy.
    public: BuoyState state = BuoyState::NEVER_ENGAGED;

    /// \brief Pointer to the vehicle that interacts with the buoy.
    public: gazebo::physics::ModelPtr vehicleModel;

    /// \brief A collection of virtual gates around the buoy.
    public: std::vector<VirtualGate> virtualGates;

    /// \brief The number of virtual gates currently crossed.
    public: unsigned int numVirtualGatesCrossed = 0u;
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
  /// \param[in] _modelName The name of the buoy's model.
  /// \param[in] _linkName The name of the main buoy's link.
  /// \param[in] _goal The goal associated to this buoy.
  /// \return True when the buoy has been registered or false otherwise.
  private: bool AddBuoy(const std::string &_modelName,
                        const std::string &_linkName,
                        const std::string &_goal);

  /// \brief Callback executed at every world update.
  private: void Update();

  /// \brief Publish a new ROS message with the animal locations.
  private: void PublishAnimalLocations();

  /// \brief Compute the total bonus achieved.
  /// \return The time bonus in seconds.
  private: double TimeBonus() const;

  // Documentation inherited.
  private: void OnCollision() override;

  // Documentation inherited.
  private: virtual void OnFinished() override;

  /// \brief All the buoys.
  private: std::vector<Buoy> buoys;

  /// \brief Pointer to the update event connection.
  private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief The name of the topic where the animal locations are published.
  private: std::string animalsTopic = "/vrx/wildlife_animals";

  /// \brief Time bonus granted for each succcesfuly goal achieved.
  private: double timeBonus = 30.0;

  /// \brief When the vehicle is between the buoy and this distance, the vehicle
  /// engages with the buoy.
  private: double engagementDistance = 10.0;

  /// \brief ROS node handle.
  private: std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief Publisher for the animal locations.
  private: ros::Publisher animalsPub;

  /// \brief True when a vehicle collision is detected.
  private: std::atomic<bool> collisionDetected{false};
};

#endif
