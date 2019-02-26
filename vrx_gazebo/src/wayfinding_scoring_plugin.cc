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
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <cmath>
#include <gazebo/common/Console.hh>
#include <gazebo/common/SphericalCoordinates.hh>
#include <gazebo/physics/Model.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include "vrx_gazebo/wayfinding_scoring_plugin.hh"

/////////////////////////////////////////////////
WayfindingScoringPlugin::WayfindingScoringPlugin()
{
  gzmsg << "Wayfinding scoring plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void WayfindingScoringPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  ScoringPlugin::Load(_world, _sdf);
  this->sdf = _sdf;
  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  // A waypoints element is required.
  if (!this->sdf->HasElement("waypoints"))
  {
    gzerr << "Unable to find <waypoints> element in SDF." << std::endl;
    return;
  }
  auto waypointsElem = this->sdf->GetElement("waypoints");

  // We need at least one waypoint
  if (!waypointsElem->HasElement("waypoint"))
  {
    gzerr << "Unable to find <waypoint> element in SDF." << std::endl;
    return;
  }
  auto waypointElem = waypointsElem->GetElement("waypoint");

  while (waypointElem)
  {
    ignition::math::Vector3d latlonyaw =
      waypointElem->Get<ignition::math::Vector3d>("pose");

    // Convert lat/lon to local
    //  snippet from UUV Simulator SphericalCoordinatesROSInterfacePlugin.cc
    ignition::math::Vector3d scVec(latlonyaw.X(), latlonyaw.Y(), 0.0);

#if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Vector3d cartVec =
        _world->SphericalCoords()->LocalFromSpherical(scVec);
#else
      ignition::math::Vector3d cartVec =
        _world->GetSphericalCoordinates()->LocalFromSpherical(scVec);
#endif

    cartVec.Z() = latlonyaw.Z();

    // Set up relevant vectors
    this->sphericalWaypoints.push_back(latlonyaw);
    this->localWaypoints.push_back(cartVec);

    // Print some debugging messages
    gzmsg << "Waypoint, Spherical: Lat = " << latlonyaw.X()
          << " Lon = " << latlonyaw.Y() << std::endl;
    gzmsg << "Waypoint, Local: X = " << cartVec.X()
          << " Y = " << cartVec.Y() << " Yaw = " << cartVec.Z() << std::endl;

    waypointElem = waypointElem->GetNextElement("waypoint");
  }

  // Setup ROS node and publisher
  this->rosNode.reset(new ros::NodeHandle());
  this->waypointsPub =
    this->rosNode->advertise<geographic_msgs::GeoPoseStamped>(
      this->waypointsTopic, 10);

  this->minErrorsPub =
    this->rosNode->advertise<std_msgs::Float64MultiArray>(
      this->minErrorsTopic, 100);

  this->meanErrorPub =
    this->rosNode->advertise<std_msgs::Float64>(
      this->meanErrorTopic, 100);

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&WayfindingScoringPlugin::Update, this));
}

//////////////////////////////////////////////////
void WayfindingScoringPlugin::Update()
{
  // The vehicle might not be ready yet, let's try to get it.
  if (!this->vehicleModel)
  {
    this->vehicleModel = this->world->GetModel(this->vehicleName);
    if (!this->vehicleModel)
      return;
  }

  std_msgs::Float64MultiArray minErrorsMsg;
  std_msgs::Float64 meanErrorMsg;

  const auto robotPose = this->vehicleModel->GetWorldPose();
  double currentHeading = robotPose.rot.GetAsEuler().z;

  double currentTotalError = 0;

  for (unsigned i = 0; i < this->localWaypoints.size(); ++i)
  {
    const ignition::math::Vector3d wp = this->localWaypoints[i];
    double dx   =  wp.X() - robotPose.pos.x;
    double dy   =  wp.Y() - robotPose.pos.y;
    double dhdg =  wp.Z() - currentHeading;
    double poseError =  sqrt(pow(dx, 2) + pow(dy, 2) + pow(dhdg, 2));

    // If this is the first time through, minError == poseError
    if (i == this->minErrors.size())
    {
      this->minErrors.push_back(poseError);
    }

    // If poseError is smaller than the minimum, update the minimum
    if (poseError < this->minErrors.at(i))
    {
      this->minErrors.at(i) = poseError;
    }

    // add current minimum to current total error
    currentTotalError += this->minErrors.at(i);
  }

  this->meanError = currentTotalError / this->localWaypoints.size();

  // Set up multi array dimensions
  minErrorsMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  minErrorsMsg.layout.dim[0].label = "minimum errors";
  minErrorsMsg.layout.dim[0].size = this->localWaypoints.size();
  minErrorsMsg.layout.dim[0].stride = this->localWaypoints.size();

  minErrorsMsg.data = this->minErrors;
  meanErrorMsg.data = this->meanError;

  this->minErrorsPub.publish(minErrorsMsg);
  this->meanErrorPub.publish(meanErrorMsg);

  this->ScoringPlugin::SetScore(this->meanError);
}

//////////////////////////////////////////////////
void WayfindingScoringPlugin::PublishGoal()
{
  gzmsg << "Publishing Waypoints" << std::endl;
  geographic_msgs::GeoPoseStamped wp_msg;

  for (auto wp : this->sphericalWaypoints)
  {
    wp_msg.pose.position.latitude  = wp.X();
    wp_msg.pose.position.longitude = wp.Y();
    wp_msg.pose.position.altitude  = 0.0;

    const ignition::math::Quaternion<double> orientation(0.0, 0.0, wp.Z());

    wp_msg.pose.orientation.x = orientation.X();
    wp_msg.pose.orientation.y = orientation.Y();
    wp_msg.pose.orientation.z = orientation.Z();
    wp_msg.pose.orientation.w = orientation.W();

    wp_msg.header.stamp = ros::Time::now();

    this->waypointsPub.publish(wp_msg);
  }
}

//////////////////////////////////////////////////
void WayfindingScoringPlugin::OnReady()
{
  gzmsg << "OnReady" << std::endl;

  this->PublishGoal();
}


//////////////////////////////////////////////////
void WayfindingScoringPlugin::OnRunning()
{
  gzmsg << "OnRunning" << std::endl;
}

//////////////////////////////////////////////////
void WayfindingScoringPlugin::OnFinished()
{
  gzmsg << "OnFinished" << std::endl;
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(WayfindingScoringPlugin)
