/*
 * Copyright (C) 2017  Brian Bingham
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
#include <functional>
#include <string>
#include <gazebo/common/Console.hh>

#include "usv_gazebo_plugins/usv_gazebo_wind_plugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
UsvWindPlugin::UsvWindPlugin()
{
}

//////////////////////////////////////////////////
void UsvWindPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  std::string linkName;
  physics::ModelPtr model = _parent;

  // Retrieve model parameters from SDF
  if (!_sdf->HasElement("bodyName") ||
      !_sdf->GetElement("bodyName")->GetValue())
  {
    this->link = model->GetLink();
    linkName = this->link->GetName();
    gzerr << "Did not find SDF parameter bodyName" << std::endl;
  }
  else
  {
    linkName = _sdf->GetElement("bodyName")->Get<std::string>();
    this->link = model->GetLink(linkName);

    gzmsg << "Found SDF parameter bodyName as [" << linkName << "]"
          << std::endl;
  }
  if (!this->link)
  {
    gzerr << "usv_gazebo_wind_plugin error: bodyName: [" << linkName << "] does"
          << " not exist" << std::endl;
    return;
  }

  gzmsg << "USV Model Link Name = " << linkName << std::endl;

  if (_sdf->HasElement("wind_velocity_vector"))
  {
    this->windVelocity =
      _sdf->GetElement("wind_velocity_vector")->Get<ignition::math::Vector3d>();
  }

  gzmsg << "Wind velocity vector = " << this->windVelocity.X() << " , "
        << this->windVelocity.Y() << " , " << this->windVelocity.Z()
        << std::endl;

  if (_sdf->HasElement("wind_coeff_vector"))
  {
    windCoeff =
      _sdf->GetElement("wind_coeff_vector")->Get<ignition::math::Vector3d>();
  }

  gzmsg << "Wind coefficient vector = " << windCoeff.X() << " , "
        << windCoeff.Y() << " , " << windCoeff.Z() << std::endl;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&UsvWindPlugin::Update, this));
}

//////////////////////////////////////////////////
void UsvWindPlugin::Update()
{
  // Transform wind from world coordinates to body coordinates
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Vector3d relativeWind =
    this->link->WorldPose().Rot().Inverse().RotateVector(this->windVelocity);
#else
  ignition::math::Vector3d relativeWind =
    this->link->GetWorldPose().rot.Ign().Inverse().RotateVector(
    this->windVelocity);
#endif        
  // Calculate apparent wind
#if GAZEBO_MAJOR_VERSION >= 8  
  ignition::math::Vector3d apparentWind =
    relativeWind - this->link->RelativeLinearVel();
#else
  ignition::math::Vector3d apparentWind = relativeWind
    - this->link->GetRelativeLinearVel().Ign();
#endif
  // gzdbg << "Relative wind: " << relativeWind << std::endl;
  // gzdbg << "Apparent wind: " << apparentWind << std::endl;

  // Calculate wind force - body coordinates
  ignition::math::Vector3d windForce(
    windCoeff.X() * relativeWind.X() * abs(relativeWind.X()),
    windCoeff.Y() * relativeWind.Y() * abs(relativeWind.Y()),
    -2.0 * windCoeff.Z() * relativeWind.X() * relativeWind.Y());

  // Add forces/torques to link at CG
  this->link->AddRelativeForce(
    ignition::math::Vector3d(windForce.X(), windForce.Y(), 0.0));
  this->link->AddRelativeTorque(
    ignition::math::Vector3d(0.0, 0.0, windForce.Z()));
}

GZ_REGISTER_MODEL_PLUGIN(UsvWindPlugin);
