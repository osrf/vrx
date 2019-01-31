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

#include <cmath>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include "std_msgs/String.h"
#include <sstream>
#include <ignition/math/Pose3.hh>
#include <gazebo/common/common.hh>
#include "vmrc_gazebo/stationkeeping_scoring_plugin.hh"

/////////////////////////////////////////////////
StationkeepingScoringPlugin::StationkeepingScoringPlugin()
{
  gzmsg << "Stationkeeping scoring plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void StationkeepingScoringPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  ScoringPlugin::Load(_world, _sdf);

  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  this->sdf = _sdf;
  this->rosNode.reset(new ros::NodeHandle());
  this->goalPub = this->rosNode->advertise<geographic_msgs::GeoPoseStamped>(this->topic, 10);

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&StationkeepingScoringPlugin::Update, this));
}

//////////////////////////////////////////////////
void StationkeepingScoringPlugin::Update()
{
  // The vehicle might not be ready yet, let's try to get it.
  if (!this->vehicleModel)
  {
    this->vehicleModel = this->world->GetModel(this->vehicleName);
    if (!this->vehicleModel)
      return;
  }

  const auto robotPose = this->vehicleModel->GetWorldPose();
}

//////////////////////////////////////////////////
void StationkeepingScoringPlugin::OnReady()
{
  gzmsg << "OnReady" << std::endl;

}

//////////////////////////////////////////////////
geographic_msgs::GeoPoseStamped StationkeepingScoringPlugin::GetGoalFromSDF()
{
  gzmsg << "Getting Goal coordinates" << std::endl;
  geographic_msgs::GeoPoseStamped goal;
  ignition::math::Pose3<double> pose(0,0,0,0,0,0);

if (!this->sdf->HasElement("goal_pose"))
{
  gzerr << "Unable to find <goal_pose> element in SDF." << std::endl;
  gzerr << "Using default pose: 0 0 0 0 0 0" << std::endl;
} else {
  pose = this->sdf->Get<ignition::math::Pose3<double>>("goal_pose");
}

  // convert local position to lat/lon/alt
  ignition::math::SphericalCoordinates::SurfaceType st =
  ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::SphericalCoordinates sc(st);

  ignition::math::Vector3d coordinates = sc.SphericalFromLocalPosition(pose.Pos());

  // populating GeoPoseStamped... must be a better way?
  goal.pose.position.latitude  = coordinates[0];
  goal.pose.position.longitude = coordinates[1];
  goal.pose.position.altitude  = coordinates[2];

  const ignition::math::Quaternion<double> orientation = pose.Rot();

  goal.pose.orientation.x = orientation.X();
  goal.pose.orientation.y = orientation.Y();
  goal.pose.orientation.z = orientation.Z();
  goal.pose.orientation.w = orientation.W();

  goal.header.stamp = ros::Time::now();
  return goal;
}
//////////////////////////////////////////////////
void StationkeepingScoringPlugin::OnRunning()
{
  gzmsg << "OnRunning" << std::endl;

  this->goalPub.publish(this->GetGoalFromSDF());
}

//////////////////////////////////////////////////
void StationkeepingScoringPlugin::OnFinished()
{
  gzmsg << "OnFinished" << std::endl;
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(StationkeepingScoringPlugin)
