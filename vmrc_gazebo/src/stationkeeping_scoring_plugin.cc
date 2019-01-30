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
void StationkeepingScoringPlugin::OnRunning()
{
  gzmsg << "OnRunning" << std::endl;
}

//////////////////////////////////////////////////
void StationkeepingScoringPlugin::OnFinished()
{
  gzmsg << "OnFinished" << std::endl;
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(StationkeepingScoringPlugin)
