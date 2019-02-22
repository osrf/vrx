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
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include <sstream>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <gazebo/common/common.hh>
#include "vmrc_gazebo/wayfinding_scoring_plugin.hh"
#include "gazebo/common/SphericalCoordinates.hh"


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
    
    this->sphericalWaypoints.push_back(waypointElem->Get<ignition::math::Vector3d>("pose"));

    waypointElem = waypointElem->GetNextElement("waypoint"); 
  }

  
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

}


//////////////////////////////////////////////////
void WayfindingScoringPlugin::PublishGoal()
{
  gzmsg << "Publishing Goal coordinates" << std::endl;
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
