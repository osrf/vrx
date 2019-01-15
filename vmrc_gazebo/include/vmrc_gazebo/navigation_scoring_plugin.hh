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
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/math/Pose.hh>
#include <sdf/sdf.hh>

/// \brief A plugin for computing the score of the navigation task.
class NavigationScoringPlugin : public gazebo::WorldPlugin
{
  // Constructor.
  public: NavigationScoringPlugin();

  // Documentation inherited.
  public: void Load(gazebo::physics::WorldPtr _world,
                    sdf::ElementPtr _sdf);

  /// \brief Callback executed at every world update.
  private: void Update();

  /// \brief Is the given robot pose "in" the given gate pose?
  /// \param _robotWorldPose Pose of the robot, in the world frame
  /// \param _gateWorldPose Pose of the gate, in the world frame
  /// \param _gateWorldPose Width of the gate
  /// \return If not "in" the gate, return 0; else return -1 if "before" the
  ///         gate, 1 if "after" the gate.
  private: int IsPoseInGate(const gazebo::math::Pose &_robotWorldPose,
                            const gazebo::math::Pose &_gateWorldPose,
                            double _gateWidth);

  /// \brief A gate.
  struct Gate
  {
    // The center of the gate.
    gazebo::math::Pose pose;

    // The width of the gate.
    double width;
  };

  /// \brief All the gates.
  private: std::vector<Gate> gates;

  /// \brief Pointer to the update event connection.
  private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief A world pointer.
  private: gazebo::physics::WorldPtr world;

  /// \brief Gate states.
  private: std::vector<int8_t> gateStates;
};

#endif