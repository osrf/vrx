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
/// This plugin derives from the generic ScoringPlugin class. Refer to that
/// plugin for an explanation of the four states defined (Initial, Ready, 
/// Running and Finished) as well as other required SDF elements.
///
/// This plugin publishes a goal pose to a topic when it enters the Ready
/// states. 
/// 
/// In the running state it calculates a 2D pose error distance between the
/// vehicle and the goal as well as a running RMS error of all 2D pose errors
/// calculated so far. The current 2D pose error is published to a topic
/// for pose error, and the RMS error is published to a task score topic.
/// RMS error is also set as the score using the SetScore() method inherited from
/// the parent. This causes it to also appear in the task information topic.
///
/// This plugin requires the following SDF parameters:
///
/// <goal_pose>: Optional parameter (vector type) specifying the lattitude,
/// longitude and yaw of the task goal. If not provided, all values default
/// to 0.
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

  /// \brief Publish the goal pose
  private: void PublishGoal();

  /// \brief Calculate instantaneous error 
  private: double InstantaneousError();


  /// \brief Pointer to the update event connection.
  private: gazebo::event::ConnectionPtr updateConnection;

 /// \brief The next task message to be published.
  private: vmrc_gazebo::Task taskMsg;

 /// \brief Pointer to the sdf plugin element.
  private: sdf::ElementPtr sdf;

 /// \brief Topic where the task stats are published.
  protected: std::string goalTopic = "/vmrc/task/goal";

 /// \brief Topic where 2D pose error is published 
  protected: std::string poseErrorTopic = "/vmrc/task/pose_error";

 /// \brief Topic where total RMS pose error is published.
  protected: std::string rmsErrorTopic = "/vmrc/task/rms_error";

  /// \brief ROS node handle.
  private: std::unique_ptr<ros::NodeHandle> rosNode;

 /// \brief Publisher for the goal.
  private: ros::Publisher goalPub;

 /// \brief Publisher for the combined 2D pose error.
  private: ros::Publisher poseErrorPub;

 /// \brief Publisher for the current rms error.
  private: ros::Publisher rmsErrorPub;

  /// \brief Goal pose in local (Gazebo) coordiates
  private: double goalX;

  /// \brief Goal pose in local (Gazebo) coordiates
  private: double goalY;

  /// \brief Goal pose in local (Gazebo) coordiates
private: double goalYaw;
	
  /// \brief Goal pose in spherical (WGS84) coordinates
  private: double goalLat;

  /// \brief Goal pose in spherical (WGS84) coordinates
  private: double goalLon;

  /// \brief combined 2D pose error (distance and yaw)
  private: double poseError;

  /// \brief number of instant pose error scores calculated so far 
  private: double sampleCount;

  /// \brief sum of squared error scores calculated so far 
  private: double totalSquaredError;


  /// \brief Cumulative 2D RMS error in meters  
  private: double rmsError;
	
};

#endif
