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
  auto redPose = this->redBuoyModel->GetWorldPose();
  auto greenPose = this->greenBuoyModel->GetWorldPose();

  // Unit vector from the green buoy to the red one.
  auto v1 = redPose.pos - greenPose.pos;
  v1.Normalize();

  // Unit vector perpendicular to v1 in the direction we like to cross gates.
  auto v2 = gazebo::math::Vector3::UnitZ.Cross(v1);

  // This is the center point of the gate.
  auto middle = (redPose.pos + greenPose.pos) / 2.0;

  // Yaw of the gate in world coordinates.
  auto yaw = atan2(v2.y, v2.x);

  // The updated pose.
  this->pose.Set(middle, gazebo::math::Vector3(0, 0, yaw));

  // The updated width.
  this->width = redPose.pos.Distance(greenPose.pos);
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
  GZ_ASSERT(_world, "NavigationScoringPlugin::Load(): NULL world pointer");
  GZ_ASSERT(_sdf,   "NavigationScoringPlugin::Load(): NULL _sdf pointer");

  this->world = _world;

  // This is a required element.
  if (!_sdf->HasElement("gates"))
  {
    gzerr << "Unable to find any gates" << std::endl;
    return;
  }

  // Parse all the gates.
  auto const &gatesElem = _sdf->GetElement("gates");
  if (!this->ParseGates(gatesElem))
  {
    gzerr << "Score disabled" << std::endl;
    return;
  }

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
    gzerr << "Unable to find any gate" << std::endl;
    return false;
  }

  auto gateElem = _sdf->GetElement("gate");

  // Parse a new gate.
  while (gateElem)
  {
    // The red buoy's name.
    if (!gateElem->HasElement("red"))
    {
      gzerr << "Unable to find red element" << std::endl;
      return false;
    }

    std::string redBuoyName = gateElem->Get<std::string>("red");

    // The green buoy's name.
    if (!gateElem->HasElement("green"))
    {
      gzerr << "Unable to find green element" << std::endl;
      return false;
    }

    std::string greenBuoyName = gateElem->Get<std::string>("green");

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

  // Initialize the state of the gate.
  this->gateStates.push_back(0);

  return true;
}

//////////////////////////////////////////////////
void NavigationScoringPlugin::Update()
{
  auto robotModel = this->world->GetModel("wamv");
  if (!robotModel)
    return;

  auto robotPose = robotModel->GetWorldPose();

  // Update the state of all gates.
  for (auto i = 0; i < this->gates.size(); ++i)
  {
    // Ignore all gates that have been crossed.
    if (this->gateStates[i] == 2)
      continue;

    // Get the next gate.
    auto &gate = this->gates[i];

    // Update this gate (in case it moved).
    gate.Update();

    // Check if we have crossed this gate.
    auto currentState = this->IsPoseInGate(robotPose, gate);
    if (currentState == 1 && gateStates[i] == -1)
    {
      currentState = 2;
      gzmsg << "Gate " << i << " crossed!" << std::endl;
    }

    gateStates[i] = currentState;
  }
}

/////////////////////////////////////////////////
uint8_t NavigationScoringPlugin::IsPoseInGate(
    const gazebo::math::Pose &_robotWorldPose, const Gate &_gate) const
{
  // Transform to gate frame.
  gazebo::math::Vector3 robotLocalPosition =
    _gate.pose.rot.GetInverse().RotateVector(_robotWorldPose.pos -
    _gate.pose.pos);

  // Are we within the width?
  if (fabs(robotLocalPosition.y) <= _gate.width / 2.0)
    return (robotLocalPosition.x >= 0.0) ? 1 : -1;
  else
    return 0;
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(NavigationScoringPlugin)
