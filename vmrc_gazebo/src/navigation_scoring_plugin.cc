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

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include "vmrc_gazebo/navigation_scoring_plugin.hh"

using namespace gazebo;

/////////////////////////////////////////////////
NavigationScoringPlugin::NavigationScoringPlugin()
{
  gzmsg << "Navigation scoring plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void NavigationScoringPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "CommsBrokerPlugin world pointer is NULL");
  GZ_ASSERT(_sdf,   "CommsBrokerPlugin::Load() error: _sdf pointer is NULL");

  this->world = _world;

  // This is a required element.
  if (!_sdf->HasElement("gates"))
  {
    gzerr << "Unable to find any gates" << std::endl;
    return;
  }

  auto const &gatesElem = _sdf->GetElement("gates");

  // We need at least one gate.
  if (!gatesElem->HasElement("gate"))
  {
    gzerr << "Unable to find any gate" << std::endl;
    return;
  }

  auto gateElem = gatesElem->GetElement("gate");

  // Parse a new gate.
  while (gateElem)
  {
    // The red buoy.
    if (!gateElem->HasElement("red"))
    {
      gzerr << "Unable to find red element" << std::endl;
      return;
    }

    std::string redModelName = gateElem->Get<std::string>("red");
    gazebo::physics::ModelPtr redModel = _world->GetModel(redModelName);

    // Sanity check: Make sure that the model exists.
    if (!redModel)
    {
      gzerr << "Unable to find model [" << redModelName << "]" << std::endl;
      return;
    }

    // The green buoy.
    if (!gateElem->HasElement("green"))
    {
      gzerr << "Unable to find green element" << std::endl;
      return;
    }

    std::string greenModelName = gateElem->Get<std::string>("green");
    gazebo::physics::ModelPtr greenModel = _world->GetModel(greenModelName);

    // Sanity check: Make sure that the model exists.
    if (!greenModel)
    {
      gzerr << "Unable to find model [" << greenModelName << "]" << std::endl;
      return;
    }

    // Create a new gate and save it.
    auto redPose = redModel->GetWorldPose();
    auto greenPose = greenModel->GetWorldPose();
    auto gatePose = redPose - greenPose;
  
    std::cout << "Red: " << redPose << std::endl;
    std::cout << "Green: " << greenPose << std::endl;

    auto v = redPose.pos - greenPose.pos;
    v.Normalize();
    auto v2 = v.Cross(gazebo::math::Vector3::UnitZ);
    auto middle = (redPose.pos + greenPose.pos) / 2.0;

    auto yaw = atan2(v2.y, v2.x);

    std::cout << "Gate: \n" << "  point: " << middle << "\n"
              << "  vector: " << v2 << std::endl
              << "  angle: " << yaw * 180 / 3.14159 << std::endl;

    this->gates.push_back({{middle.x, middle.y, middle.z, 0, 0, yaw},
                           redPose.pos.Distance(greenPose.pos)});
    this->gateStates.push_back(0);

    // Parse the next gate.
    gateElem = gateElem->GetNextElement("gate");
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&NavigationScoringPlugin::Update, this));
}

//////////////////////////////////////////////////
void NavigationScoringPlugin::Update()
{
  auto robotModel = this->world->GetModel("wamv");
  if (!robotModel)
    return;

  auto robotPose = robotModel->GetWorldPose();
  auto gate = this->gates.front();
  if (this->IsPoseInGate(robotPose, gate.pose, gate.width) == 1)
    std::cout << "After 1st gate!" << std::endl;
}

/////////////////////////////////////////////////
int NavigationScoringPlugin::IsPoseInGate(
    const gazebo::math::Pose &_robotWorldPose,
    const gazebo::math::Pose &_gateWorldPose, double _gateWidth)
{
  // Transform to gate frame.
  gazebo::math::Vector3 robotLocalPosition =
    _gateWorldPose.rot.GetInverse().RotateVector(_robotWorldPose.pos -
    _gateWorldPose.pos);

  // Are we within the width?
  if (fabs(robotLocalPosition.y) <= _gateWidth / 2.0)
    return (robotLocalPosition.x >= 0.0) ? 1 : -1;
  else
    return 0;
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(NavigationScoringPlugin)
