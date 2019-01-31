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

#ifndef VMRC_GAZEBO_STATIONKEEPING_SCORING_PLUGIN_HH_
#define VMRC_GAZEBO_STATIONKEEPING_SCORING_PLUGIN_HH_

#include <string>
#include <vector>
#include <gazebo/common/Events.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>
#include "geographic_msgs/GeoPoseStamped.h"
#include "vmrc_gazebo/scoring_plugin.hh"

/// \brief A plugin for computing the score of the station keeping task.
/// This plugin derives from the generic ScoringPlugin class. Check out that
/// plugin for other required SDF elements.
/// This plugin requires the following SDF parameters:
///
class StationkeepingScoringPlugin : public ScoringPlugin
{

  // Constructor.
  public: StationkeepingScoringPlugin();

  // Documentation inherited.
  public: void Load(gazebo::physics::WorldPtr _world,
                    sdf::ElementPtr _sdf);

  /// \brief Callback executed at every world update.
  private: void Update();

  // Documentation inherited.
  private: void OnReady() override;

  // Documentation inherited.
  private: void OnRunning() override;

  // Documentation inherited.
  private: void OnFinished() override;

  private: geographic_msgs::GeoPoseStamped GetGoalFromSDF();

  /// \brief Pointer to the update event connection.
  private: gazebo::event::ConnectionPtr updateConnection;

 /// \brief The next task message to be published.
  private: vmrc_gazebo::Task taskMsg;

 /// \brief Pointer to the sdf plugin element.
  private: sdf::ElementPtr sdf;

 /// \brief Topic where the task stats are published.
  protected: std::string topic = "/vmrc/task/goal";

  /// \brief ROS node handle.
  private: std::unique_ptr<ros::NodeHandle> rosNode;

 /// \brief Publisher for the task state.
  private: ros::Publisher goalPub;

};

#endif
