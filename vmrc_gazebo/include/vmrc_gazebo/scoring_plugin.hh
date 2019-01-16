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

#ifndef VMRC_GAZEBO_SCORING_PLUGIN_HH_
#define VMRC_GAZEBO_SCORING_PLUGIN_HH_

#include <ros/ros.h>
#include <cstdint>
#include <memory>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>

/// \brief A plugin for ...
class ScoringPlugin : public gazebo::WorldPlugin
{
  /// \brief Class constructor.
  public: ScoringPlugin();

  // Documentation inherited.
  public: void Load(gazebo::physics::WorldPtr _world,
                    sdf::ElementPtr _sdf);

  /// \brief Callback executed at every world update.
  private: void Update();

  /// \brief A world pointer.
  protected: gazebo::physics::WorldPtr world;

  /// \brief The name of the task.
  protected: std::string taskName;

  /// \brief The maximum number of seconds allowed to solve this task.
  protected: uint32_t maxTime;

  /// \brief Pointer to the update event connection.
  protected: gazebo::event::ConnectionPtr updateConnection;

  /// \brief ROS node handle.
  protected: std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief Publisher for the task state.
  protected: ros::Publisher taskStatePub;
};

#endif
