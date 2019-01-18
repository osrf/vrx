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
#include "vmrc_gazebo/navigation_scoring_plugin.hh"

/////////////////////////////////////////////////
NavigationScoringPlugin::Gate::Gate(
    const gazebo::physics::ModelPtr _redBuoyModel,
    const gazebo::physics::ModelPtr _greenBuoyModel)
  : redBuoyModel(_redBuoyModel),
    greenBuoyModel(_greenBuoyModel)
{
  this->Update();
}

/////////////////////////////////////////////////
void NavigationScoringPlugin::Gate::Update()
{
  if (!this->redBuoyModel || !this->greenBuoyModel)
    return;

  // The pose of the buoys delimiting the gate.
  const auto redPose = this->redBuoyModel->GetWorldPose();
  const auto greenPose = this->greenBuoyModel->GetWorldPose();

  // Unit vector from the green buoy to the red one.
  auto v1 = redPose.pos - greenPose.pos;
  v1.Normalize();

  // Unit vector perpendicular to v1 in the direction we like to cross gates.
  const auto v2 = gazebo::math::Vector3::UnitZ.Cross(v1);

  // This is the center point of the gate.
  const auto middle = (redPose.pos + greenPose.pos) / 2.0;

  // Yaw of the gate in world coordinates.
  const auto yaw = atan2(v2.y, v2.x);

  // The updated pose.
  this->pose.Set(middle, gazebo::math::Vector3(0, 0, yaw));

  // The updated width.
  this->width = redPose.pos.Distance(greenPose.pos);
}

/////////////////////////////////////////////////
NavigationScoringPlugin::GateState NavigationScoringPlugin::Gate::IsPoseInGate(
    const gazebo::math::Pose &_robotWorldPose) const
{
  // Transform to gate frame.
  const gazebo::math::Vector3 robotLocalPosition =
    this->pose.rot.GetInverse().RotateVector(_robotWorldPose.pos -
    this->pose.pos);

  // Are we within the width?
  if (fabs(robotLocalPosition.y) <= this->width / 2.0)
  {
    if (robotLocalPosition.x >= 0.0)
      return GateState::VEHICLE_AFTER;
    else
      return GateState::VEHICLE_BEFORE;
  }
  else
    return GateState::VEHICLE_OUTSIDE;
}


/////////////////////////////////////////////////
NavigationScoringPlugin::NavigationScoringPlugin()
{
  gzmsg << "Navigation scoring plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void NavigationScoringPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  ScoringPlugin::Load(_world, _sdf);

  // This is a required element.
  if (!_sdf->HasElement("vehicle"))
  {
    gzerr << "Unable to find <vehicle> element in SDF." << std::endl;
    return;
  }
  this->vehicleName = _sdf->Get<std::string>("vehicle");

  // This is a required element.
  if (!_sdf->HasElement("gates"))
  {
    gzerr << "Unable to find <gates> element in SDF." << std::endl;
    return;
  }

  // Parse all the gates.
  auto const &gatesElem = _sdf->GetElement("gates");
  if (!this->ParseGates(gatesElem))
  {
    gzerr << "Score has been disabled" << std::endl;
    return;
  }

  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&NavigationScoringPlugin::Update, this));
}

//////////////////////////////////////////////////
bool NavigationScoringPlugin::ParseGates(sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_sdf, "NavigationScoringPlugin::ParseGates(): NULL _sdf pointer");

  // We need at least one gate.
  if (!_sdf->HasElement("gate"))
  {
    gzerr << "Unable to find <gate> element in SDF." << std::endl;
    return false;
  }

  auto gateElem = _sdf->GetElement("gate");

  // Parse a new gate.
  while (gateElem)
  {
    // The red buoy's name.
    if (!gateElem->HasElement("red"))
    {
      gzerr << "Unable to find <red> element in SDF." << std::endl;
      return false;
    }

    const std::string redBuoyName = gateElem->Get<std::string>("red");

    // The green buoy's name.
    if (!gateElem->HasElement("green"))
    {
      gzerr << "Unable to find <green> element in SDF." << std::endl;
      return false;
    }

    const std::string greenBuoyName = gateElem->Get<std::string>("green");

    if (!this->AddGate(redBuoyName, greenBuoyName))
      return false;

    // Parse the next gate.
    gateElem = gateElem->GetNextElement("gate");
  }

  return true;
}

//////////////////////////////////////////////////
bool NavigationScoringPlugin::AddGate(const std::string &_redBuoyName,
    const std::string &_greenBuoyName)
{
  gazebo::physics::ModelPtr redBuoyModel = this->world->GetModel(_redBuoyName);

  // Sanity check: Make sure that the model exists.
  if (!redBuoyModel)
  {
    gzerr << "Unable to find model [" << _redBuoyName << "]" << std::endl;
    return false;
  }

  gazebo::physics::ModelPtr greenBuoyModel =
    this->world->GetModel(_greenBuoyName);

  // Sanity check: Make sure that the model exists.
  if (!greenBuoyModel)
  {
    gzerr << "Unable to find model [" << _greenBuoyName << "]" << std::endl;
    return false;
  }

  // Save the new gate.
  this->gates.push_back(Gate(redBuoyModel, greenBuoyModel));

  return true;
}

//////////////////////////////////////////////////
void NavigationScoringPlugin::Update()
{
  // The vehicle might not be ready yet, let's try to get it.
  if (!this->vehicleModel)
  {
    this->vehicleModel = this->world->GetModel(this->vehicleName);
    if (!this->vehicleModel)
      return;
  }

  const auto robotPose = this->vehicleModel->GetWorldPose();

  // Update the state of all gates.
  for (auto &gate : this->gates)
  {
    // Ignore all gates that have been crossed or are invalid.
    if (gate.state == GateState::CROSSED || gate.state == GateState::INVALID)
      continue;

    // Update this gate (in case it moved).
    gate.Update();

    // Check if we have crossed this gate.
    auto currentState = gate.IsPoseInGate(robotPose);
    if (currentState == GateState::VEHICLE_AFTER &&
        gate.state   == GateState::VEHICLE_BEFORE)
    {
      currentState = GateState::CROSSED;
      gzmsg << "New gate crossed!" << std::endl;
    }
    // Just checking: did we go backward through the gate?
    else if (currentState == GateState::VEHICLE_BEFORE &&
             gate.state   == GateState::VEHICLE_AFTER)
    {
      currentState = GateState::INVALID;
      gzmsg << "Went backward through gate. Invalidated!" << std::endl;
    }

    gate.state = currentState;
  }

  // gzmsg << "Elapsed: " << this->ElapsedTime() << std::endl;
  // gzmsg << "Remaining: " << this->RemainingTime() << std::endl;
}

//////////////////////////////////////////////////
void NavigationScoringPlugin::OnReady()
{
  gzmsg << "OnReady" << std::endl;
}

//////////////////////////////////////////////////
void NavigationScoringPlugin::OnRunning()
{
  gzmsg << "OnRunning" << std::endl;
}

//////////////////////////////////////////////////
void NavigationScoringPlugin::OnFinished()
{
  gzmsg << "OnFinished" << std::endl;
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(NavigationScoringPlugin)
