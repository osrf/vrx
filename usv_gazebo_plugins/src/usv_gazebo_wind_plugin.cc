/*

Copyright (c) 2018, Brian Bingham
All rights reserved

This file is part of the usv_gazebo_dynamics_plugin package, known as this Package.

This Package free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This Package s distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this package.  If not, see <http://www.gnu.org/licenses/>.

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
      _sdf->GetElement("wind_velocity_vector")->Get<math::Vector3>();
  }

  gzmsg << "Wind velocity vector = " << this->windVelocity.x << " , "
        << this->windVelocity.y << " , " << this->windVelocity.z << std::endl;

  if (_sdf->HasElement("wind_coeff_vector"))
  {
    windCoeff = _sdf->GetElement("wind_coeff_vector")->Get<math::Vector3>();
  }

  gzmsg << "Wind coefficient vector = " << windCoeff.x << " , " << windCoeff.y
        << " , " << windCoeff.z << std::endl;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&UsvWindPlugin::Update, this));
}

//////////////////////////////////////////////////
void UsvWindPlugin::Update()
{
  // Transform wind from world coordinates to body coordinates
  math::Vector3 relativeWind = this->link->GetWorldPose().rot.GetInverse().
    RotateVector(this->windVelocity);
  // Calculate apparent wind
  math::Vector3 apparentWind =
    relativeWind - this->link->GetRelativeLinearVel();

  // gzdbg << "Relative wind: " << relativeWind << std::endl;
  // gzdbg << "Apparent wind: " << apparentWind << std::endl;

  // Calculate wind force - body coordinates
  math::Vector3 windForce(
    windCoeff.x * relativeWind.x * abs(relativeWind.x),
    windCoeff.y * relativeWind.y * abs(relativeWind.y),
    -2.0 * windCoeff.z * relativeWind.x * relativeWind.y);

  // Add forces/torques to link at CG
  this->link->AddRelativeForce(math::Vector3(windForce.x, windForce.y, 0.0));
  this->link->AddRelativeTorque(math::Vector3(0.0, 0.0, windForce.z));
}

GZ_REGISTER_MODEL_PLUGIN(UsvWindPlugin);
