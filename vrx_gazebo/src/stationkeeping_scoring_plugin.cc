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

#include <std_msgs/Float64.h>
#include <cmath>
#include <gazebo/common/Console.hh>
#include <gazebo/common/SphericalCoordinates.hh>
#include <gazebo/physics/Model.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include "vrx_gazebo/stationkeeping_scoring_plugin.hh"

/////////////////////////////////////////////////
StationkeepingScoringPlugin::StationkeepingScoringPlugin()
{
  gzmsg << "Stationkeeping scoring plugin loaded" << std::endl;

  this->timer.Stop();
  this->timer.Reset();
}

/////////////////////////////////////////////////
void StationkeepingScoringPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  ScoringPlugin::Load(_world, _sdf);
  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  // Get lat, lon and yaw from SDF
  ignition::math::Vector3d latlonyaw(0, 0, 0);
  if (!_sdf->HasElement("goal_pose"))
  {
    ROS_ERROR("Unable to find <goal_pose> element in SDF.");
    ROS_ERROR("Using default pose: 0 0 0");
  }
  else
  {
    latlonyaw = _sdf->Get<ignition::math::Vector3d>("goal_pose");
  }
  // Store spherical 2D location
  this->goalLat = latlonyaw.X();
  this->goalLon = latlonyaw.Y();

  // Convert lat/lon to local
  // Snippet from UUV Simulator SphericalCoordinatesROSInterfacePlugin.cc
  ignition::math::Vector3d scVec(this->goalLat, this->goalLon, 0.0);

#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Vector3d cartVec =
  _world->SphericalCoords()->LocalFromSpherical(scVec);
#else
  ignition::math::Vector3d cartVec =
  _world->GetSphericalCoordinates()->LocalFromSpherical(scVec);
#endif

  // Store local 2D location and yaw
  this->goalX = cartVec.X();
  this->goalY = cartVec.Y();
  this->goalYaw = latlonyaw.Z();

  // Print some debugging messages
  gzmsg << "StationKeeping Goal, Spherical: Lat = " << this->goalLat
        << " Lon = " << this->goalLon << std::endl;
  gzmsg << "StationKeeping Goal, Local: X = " << this->goalX
        << " Y = " << this->goalY << " Yaw = " << this->goalYaw << std::endl;

  // Setup ROS node and publisher
  this->rosNode.reset(new ros::NodeHandle());
  this->goalPub = this->rosNode->advertise<geographic_msgs::GeoPoseStamped>(
    this->goalTopic, 10, true);

  this->poseErrorPub = this->rosNode->advertise<std_msgs::Float64>(
    this->poseErrorTopic, 100);
  this->rmsErrorPub  = this->rosNode->advertise<std_msgs::Float64>(
    this->rmsErrorTopic, 100);

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&StationkeepingScoringPlugin::Update, this));
}

//////////////////////////////////////////////////
void StationkeepingScoringPlugin::Update()
{
  // The vehicle might not be ready yet, let's try to get it.
  if (!this->vehicleModel)
  {
    #if GAZEBO_MAJOR_VERSION >= 8
      this->vehicleModel = this->world->ModelByName(this->vehicleName);
    #else
      this->vehicleModel = this->world->GetModel(this->vehicleName);
    #endif
    if (!this->vehicleModel)
      return;
  }

  // Nothing to do if the task is not in "running" state.
  if (this->ScoringPlugin::TaskState() != "running")
    return;

  std_msgs::Float64 poseErrorMsg;
  std_msgs::Float64 rmsErrorMsg;

  #if GAZEBO_MAJOR_VERSION >= 8
    const auto robotPose = this->vehicleModel->WorldPose();
  #else
    const auto robotPose = this->vehicleModel->GetWorldPose().Ign();
  #endif

  double currentHeading = robotPose.Rot().Euler().Z();
  double dx   =  this->goalX - robotPose.Pos().X();
  double dy   =  this->goalY - robotPose.Pos().Y();
  double dhdg =  this->goalYaw - currentHeading;

  double sqError =  pow(dx, 2) + pow(dy, 2) + pow(dhdg, 2);

  this->poseError  = sqrt(sqError);
  this->totalSquaredError += sqError;
  this->sampleCount++;

  this->rmsError = sqrt(this->totalSquaredError / this->sampleCount);

  poseErrorMsg.data = this->poseError;
  rmsErrorMsg.data = this->rmsError;

  // Publish at 1 Hz.
  if (this->timer.GetElapsed() >= gazebo::common::Time(1.0))
  {
    this->poseErrorPub.publish(poseErrorMsg);
    this->rmsErrorPub.publish(rmsErrorMsg);
    this->timer.Reset();
    this->timer.Start();
  }

  this->ScoringPlugin::SetScore(this->rmsError);
}

//////////////////////////////////////////////////
void StationkeepingScoringPlugin::PublishGoal()
{
  gzmsg << "Publishing Goal coordinates" << std::endl;
  geographic_msgs::GeoPoseStamped goal;

  // populating GeoPoseStamped... must be a better way?
  goal.pose.position.latitude  = this->goalLat;
  goal.pose.position.longitude = this->goalLon;
  goal.pose.position.altitude  = 0.0;

  const ignition::math::Quaternion<double> orientation(0.0, 0.0, this->goalYaw);

  goal.pose.orientation.x = orientation.X();
  goal.pose.orientation.y = orientation.Y();
  goal.pose.orientation.z = orientation.Z();
  goal.pose.orientation.w = orientation.W();

  goal.header.stamp = ros::Time::now();

  this->goalPub.publish(goal);
}

//////////////////////////////////////////////////
void StationkeepingScoringPlugin::OnReady()
{
  gzmsg << "OnReady" << std::endl;

  this->PublishGoal();
}

//////////////////////////////////////////////////
void StationkeepingScoringPlugin::OnRunning()
{
  gzmsg << "OnRunning" << std::endl;

  this->timer.Start();
}



// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(StationkeepingScoringPlugin)
