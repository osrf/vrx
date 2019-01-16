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

#ifndef VMRC_GAZEBO_NAVIGATION_SCORING_PLUGIN_HH_
#define VMRC_GAZEBO_NAVIGATION_SCORING_PLUGIN_HH_

#include <ros/ros.h>
#include <string>
#include <vector>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>

/// \brief A plugin for computing the score of the navigation task.
/// This plugin requires the following SDF parameters:
///
/// <vehicle>: The name of the vehicle that should cross the gates. 
/// <gates>: Specifies the collection of gates delimiting the course.
///
///   Each gate accepts the following elements:
///
///   <gate>: A gate is delimited by a red and green buoy:
///
///      <red>: The name of the red buoy.
///      <green> The name of the green buoy,
///
/// Here's an example:
/// <plugin name="navigation_scoring_plugin"
///         filename="libnavigation_scoring_plugin.so">
///   <vehicle>wamv</vehicle>
///   <gates>
///     <gate>
///       <red>red_bound_0</red>
///       <green>green_bound_0</green>
///     </gate>
///     <gate>
///       <red>red_bound_1</red>
///       <green>green_bound_1</green>
///     </gate>
///     <gate>
///       <red>red_bound_2</red>
///       <green>green_bound_2</green>
///     </gate>
///     <gate>
///       <red>red_bound_3</red>
///       <green>green_bound_3</green>
///     </gate>
///     <gate>
///       <red>red_bound_4</red>
///       <green>green_bound_4</green>
///     </gate>
///     <gate>
///       <red>red_bound_5</red>
///       <green>green_bound_5</green>
///     </gate>
///     <gate>
///       <red>red_bound_6</red>
///       <green>green_bound_6</green>
///     </gate>
///   </gates>
/// </plugin>
class NavigationScoringPlugin : public gazebo::WorldPlugin
{
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

    /// \brief Gate invalid. E.g.: if crossed in the wrong direction.
    INVALID,
  };

  /// \brief A gate that is part of the navigation challenge.
  private: class Gate
  {
    /// \brief Constructor.
    /// \param[in] _redBuoyName The red buoy's model.
    /// \param[in] _greenBuoyName The green buoy's model.
    public: Gate(const gazebo::physics::ModelPtr _redBuoyModel,
                 const gazebo::physics::ModelPtr _greenBuoyModel);

    /// \brief Where is the given robot pose with respect to the gate?
    /// \param _robotWorldPose Pose of the robot, in the world frame.
    /// \return The gate state given the current robot pose.
    public: GateState IsPoseInGate(const gazebo::math::Pose &_robotWorldPose)
        const;

    /// \brief Recalculate the pose and width of the gate.
    public: void Update();

    /// \brief The red buoy model.
    public: gazebo::physics::ModelPtr redBuoyModel;

    /// \brief The green buoy model.
    public: gazebo::physics::ModelPtr greenBuoyModel;

    /// \brief The center of the gate in the world frame. Note that the roll and
    /// pitch are ignored. Only yaw is relevant and it points into the direction
    /// in which the gate should be crossed.
    public: gazebo::math::Pose pose;

    /// \brief The width of the gate in meters.
    public: double width;

    /// \brief The state of this gate.
    public: GateState state = GateState::VEHICLE_OUTSIDE;
  };

  // Constructor.
  public: NavigationScoringPlugin();

  // Documentation inherited.
  public: void Load(gazebo::physics::WorldPtr _world,
                    sdf::ElementPtr _sdf);

  /// \brief Parse the gates from SDF.
  /// \param[in] _sdf The current SDF element.
  /// \return True when the gates were successfully parsed or false othwerwise.
  private: bool ParseGates(sdf::ElementPtr _sdf);

  /// \brief Register a new gate.
  /// \param[in] _redBuoyName The name of the red buoy.
  /// \param[in] _greenBuoyName The name of the green buoy.
  /// \return True when the gate has been registered or false otherwise.
  private: bool AddGate(const std::string &_redBuoyName,
                        const std::string &_greenBuoyName);

  /// \brief Callback executed at every world update.
  private: void Update();

  /// \brief All the gates.
  private: std::vector<Gate> gates;

  /// \brief Pointer to the update event connection.
  private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief A world pointer.
  private: gazebo::physics::WorldPtr world;

  /// \brief The name of the vehicle that should cross the gates.
  private: std::string vehicleName;

  /// \brief Pointer to the vehicle that should cross the gates.
  private: gazebo::physics::ModelPtr vehicleModel;
};

#endif
