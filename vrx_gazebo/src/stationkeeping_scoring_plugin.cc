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
  : waypointMarkers("station_keeping_marker")
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
  if (!_sdf->HasElement("goal_pose") && !_sdf->HasElement("goal_pose_cart"))
  {
    ROS_ERROR("Found neither <goal_pose> nor <goal_pose_cart> element in SDF.");
    ROS_ERROR("Using default pose: 0 0 0");
  }
  else if (_sdf->HasElement("goal_pose"))
  {
    if (_sdf->HasElement("goal_pose_cart"))
    {
      ROS_ERROR("Both goal_pose and goal_pose_cart were specified.");
      ROS_ERROR("Ignoring goal_pose_cart.");
    }

    ignition::math::Vector3d latlonyaw =
        _sdf->Get<ignition::math::Vector3d>("goal_pose");

    // Store spherical 2D location
    this->goalLat = latlonyaw.X();
    this->goalLon = latlonyaw.Y();

    // Convert lat/lon to local
    // Snippet from UUV Simulator SphericalCoordinatesROSInterfacePlugin.cc
    ignition::math::Vector3d scVec(this->goalLat, this->goalLon, 0.0);
    ignition::math::Vector3d cartVec =
      this->sc.LocalFromSphericalPosition(scVec);

    // Store local 2D location and yaw
    this->goalX = cartVec.X();
    this->goalY = cartVec.Y();
    this->goalYaw = latlonyaw.Z();
  }
  else
  {
    ignition::math::Vector3d xyz =
      _sdf->Get<ignition::math::Vector3d>("goal_pose_cart");

    // Store local 2D location
    this->goalX = xyz.X();
    this->goalY = xyz.Y();

    // Convert local to lat/lon
    // Snippet from UUV Simulator SphericalCoordinatesROSInterfacePlugin.cc
    ignition::math::Vector3d cartVec(this->goalX, this->goalY, xyz.Z());

    auto in = ignition::math::SphericalCoordinates::CoordinateType::GLOBAL;
    auto out = ignition::math::SphericalCoordinates::CoordinateType::SPHERICAL;
    auto scVec = this->sc.PositionTransform(cartVec, in, out);
    scVec.X(IGN_RTOD(scVec.X()));
    scVec.Y(IGN_RTOD(scVec.Y()));

    // Store spherical 2D location
    this->goalLat = scVec.X();
    this->goalLon = scVec.Y();
    this->goalYaw = scVec.Z();
  }

  // Print some debugging messages
  gzmsg << "StationKeeping Goal, Spherical: Lat = " << this->goalLat
        << " Lon = " << this->goalLon << std::endl;
  gzmsg << "StationKeeping Goal, Local: X = " << this->goalX
        << " Y = " << this->goalY << " Yaw = " << this->goalYaw << std::endl;

  // Setup ROS node and publisher
  this->rosNode.reset(new ros::NodeHandle());
  if (_sdf->HasElement("goal_topic"))
  {
    this->goalTopic = _sdf->Get<std::string>("goal_topic");
  }
  this->goalPub = this->rosNode->advertise<geographic_msgs::GeoPoseStamped>(
    this->goalTopic, 10, true);

  if (_sdf->HasElement("pose_error_topic"))
  {
    this->poseErrorTopic = _sdf->Get<std::string>("pose_error_topic");
  }
  this->poseErrorPub = this->rosNode->advertise<std_msgs::Float64>(
    this->poseErrorTopic, 100);

  if (_sdf->HasElement("mean_error_topic"))
  {
    this->meanErrorTopic = _sdf->Get<std::string>("mean_error_topic");
  }
  this->meanErrorPub  = this->rosNode->advertise<std_msgs::Float64>(
    this->meanErrorTopic, 100);

  if (_sdf->HasElement("head_error_on"))
    this->headErrorOn = _sdf->Get<bool>("head_error_on");

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&StationkeepingScoringPlugin::Update, this));

  if (_sdf->HasElement("markers"))
  {
    this->waypointMarkers.Load(_sdf->GetElement("markers"));
    if (this->waypointMarkers.IsAvailable())
    {
      if (!this->waypointMarkers.DrawMarker(0, this->goalX, this->goalY,
            this->goalYaw))
      {
        gzerr << "Error creating visual marker" << std::endl;
      }
    }
    else
    {
      gzwarn << "Cannot display gazebo markers (Gazebo version < 8)"
        << std::endl;
    }
  }
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
  std_msgs::Float64 meanErrorMsg;

  #if GAZEBO_MAJOR_VERSION >= 8
    const auto robotPose = this->vehicleModel->WorldPose();
  #else
    const auto robotPose = this->vehicleModel->GetWorldPose().Ign();
  #endif

  double currentHeading = robotPose.Rot().Euler().Z();
  double dx   = this->goalX - robotPose.Pos().X();
  double dy   = this->goalY - robotPose.Pos().Y();
  double dist = sqrt(pow(dx, 2) + pow(dy, 2));
  double dhdg = abs(this->goalYaw - currentHeading);
  double headError = M_PI - abs(dhdg - M_PI);

  if (this->headErrorOn)
  {
    double k    = 0.75;
    this->poseError = dist + (pow(k, dist) * headError);
  }
  else
    this->poseError = dist;
  this->totalPoseError += this->poseError;
  this->sampleCount++;

  this->meanError = this->totalPoseError / this->sampleCount;

  poseErrorMsg.data = this->poseError;
  meanErrorMsg.data = this->meanError;

  // Publish at 1 Hz.
  if (this->timer.GetElapsed() >= gazebo::common::Time(1.0))
  {
    this->poseErrorPub.publish(poseErrorMsg);
    this->meanErrorPub.publish(meanErrorMsg);
    this->timer.Reset();
    this->timer.Start();
  }

  this->ScoringPlugin::SetScore(this->meanError);
}

//////////////////////////////////////////////////
void StationkeepingScoringPlugin::PublishGoal()
{
  gzmsg << "<StationkeepingScoringPlugin> Publishing Goal coordinates"
        << std::endl;
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
  if (!this->silent)
    gzmsg << "StationkeepingScoringPlugin::OnReady" << std::endl;

  this->PublishGoal();
}

//////////////////////////////////////////////////
void StationkeepingScoringPlugin::OnRunning()
{
  if (!this->silent)
    gzmsg << "StationkeepingScoringPlugin::OnRunning" << std::endl;

  this->timer.Start();
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(StationkeepingScoringPlugin)
