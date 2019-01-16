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
#include <cstdint>
#include <string>
#include <vector>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>

/// \brief A plugin for computing the score of the navigation task.
class NavigationScoringPlugin : public gazebo::WorldPlugin
{
  /// \brief A gate that is part of the navigation challenge.
  class Gate
  {
    /// \brief Constructor.
    /// \param[in] _redBuoyName The red buoy's model.
    /// \param[in] _greenBuoyName The green buoy's model.
    public: Gate(const gazebo::physics::ModelPtr _redBuoyModel,
                 const gazebo::physics::ModelPtr _greenBuoyModel);

    // Recalculate the pose and width of the gate.
    public: void Update();

    // The red buoy.
    public: gazebo::physics::ModelPtr redBuoyModel;

    // The green buoy.
    public: gazebo::physics::ModelPtr greenBuoyModel;

    // The center of the gate.
    public: gazebo::math::Pose pose;

    // The width of the gate.
    public: double width;
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

  /// \brief Is the given robot pose "in" the given gate pose?
  /// \param _robotWorldPose Pose of the robot, in the world frame
  /// \param _gateWorldPose Pose of the gate, in the world frame
  /// \param _gateWorldPose Width of the gate
  /// \return If not "in" the gate, return 0; else return -1 if "before" the
  ///         gate, 1 if "after" the gate.
  private: uint8_t IsPoseInGate(const gazebo::math::Pose &_robotWorldPose,
                                const Gate &_gate) const;
  /// \brief All the gates.
  private: std::vector<Gate> gates;

  /// \brief Pointer to the update event connection.
  private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief A world pointer.
  private: gazebo::physics::WorldPtr world;

  /// \brief Gate states.
  ///  0: Not "in" the gate.
  /// -1: Before the gate.
  ///  1: After the gate.
  ///  2: Gate crossed!
  private: std::vector<int8_t> gateStates;
};

#endif
