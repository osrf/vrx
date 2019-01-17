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
#include <gazebo/common/Time.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>

/// \brief A plugin for ...
class ScoringPlugin : public gazebo::WorldPlugin
{
  /// \brief Class constructor.
  public: ScoringPlugin();

  // Documentation inherited.
  protected: void Load(gazebo::physics::WorldPtr _world,
                       sdf::ElementPtr _sdf);

  /// \brief Get the current score.
  protected: double Score() const;

  /// \brief Set the score.
  protected: void SetScore(double newScore);

  /// \brief Get the maximum task time (seconds).
  protected: uint32_t MaxTime() const;

  /// \brief Get the task name.
  protected: std::string TaskName() const;

  /// \brief Get the task state.
  protected: std::string TaskState() const;

  /// \brief Elapsed time since the start of the task.
  protected: gazebo::common::Time ElapsedTime() const;

  /// \brief Remaining time since the start of the task.
  protected: gazebo::common::Time RemainingTime() const;

  /// \brief Callback executed at every world update.
  private: void Update();

  /// \brief Update the state of the current task.
  /// A task can be in any of the following states:
  /// * Initial: The vehicle is lock during 5 seconds.
  /// * Running: The task is actually running.
  /// * Finished: The task has been completed or reached the maximum time.
  private: void UpdateTaskState();

  /// \brief A world pointer.
  protected: gazebo::physics::WorldPtr world;

  /// \brief The name of the task.
  protected: std::string taskName = "undefined";

  /// \brief The maximum number of seconds allowed to solve this task.
  protected: uint32_t maxTime = 0;

  /// \brief Pointer to the update event connection.
  protected: gazebo::event::ConnectionPtr updateConnection;

  /// \brief ROS node handle.
  protected: std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief Publisher for the task state.
  protected: ros::Publisher taskStatePub;

  /// \brief The score.
  protected: double score = 0.0;

  /// \brief Task start time (simulation).
  gazebo::common::Time startTime{5, 0};

  /// \brief Task finish time (simulation).
  gazebo::common::Time endTime;

  /// \brief Current time (simulation).
  gazebo::common::Time currentTime;

  /// \brief Elapsed time since the start of the task.
  gazebo::common::Time elapsedTime;

  /// \brief Remaining time since the start of the task.
  gazebo::common::Time remainingTime;

  /// \brief The task state ("initial", "running" or "finished").
  std::string taskState = "initial";
};

#endif
