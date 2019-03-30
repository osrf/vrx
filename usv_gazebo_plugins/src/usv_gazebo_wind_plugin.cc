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

  if (_sdf->HasElement("wind_direction"))
  {
    this->windDirection =
      _sdf->GetElement("wind_direction")->Get<ignition::math::Vector3d>();
    this->windDirection = this->windDirection.Normalize();
  }

  gzmsg << "Wind direction unit vector = " << this->windDirection.X() << " , "
        << this->windDirection.Y() << " , " << this->windDirection.Z()
        << std::endl;

  if (_sdf->HasElement("wind_coeff_vector"))
  {
    windCoeff =
      _sdf->GetElement("wind_coeff_vector")->Get<ignition::math::Vector3d>();
  }

  gzmsg << "Wind coefficient vector = " << windCoeff.X() << " , "
        << windCoeff.Y() << " , " << windCoeff.Z() << std::endl;

  if (_sdf->HasElement("wind_mean_velocity"))
  {
    this->windMeanVelocity =
      _sdf->GetElement("wind_mean_velocity")->Get<double>();
  }

  gzmsg << "Wind mean velocity = " << this->windMeanVelocity << std::endl;

  if (_sdf->HasElement("var_wind_gain_constants"))
  {
    this->gainConstant =
      _sdf->GetElement("var_wind_gain_constants")->Get<double>();
  }

  gzmsg << "var wind gain constants = " << this->gainConstant << std::endl;

  if (_sdf->HasElement("var_wind_time_constants"))
  {
    this->timeConstant =
      _sdf->GetElement("var_wind_time_constants")->Get<double>();
  }

  gzmsg << "var wind time constants = " << this->timeConstant << std::endl;

  // initial time and velocity
  this->previousTime = ros::Time::now().toSec();
  this->previousVarVel = 0;

  // change seed for ignition::math::Rand
  ignition::math::Rand::Seed(ros::Time::now().toSec());

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&UsvWindPlugin::Update, this));
}

//////////////////////////////////////////////////
void UsvWindPlugin::Update()
{
  this->currentTime = ros::Time::now().toSec();
  double dT= this->currentTime - this->previousTime;
  double randomDist = ignition::math::Rand::DblNormal(0, 1);
  this->currentVarVel = this->previousVarVel + (-1/this->timeConstant*(this->previousVarVel+this->gainConstant*randomDist))*dT;
  double velocity = this->currentVarVel + this->windMeanVelocity;
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Vector3d relativeWind =
    this->link->WorldPose().Rot().Inverse().RotateVector(this->windDirection*velocity);
#else
  ignition::math::Vector3d relativeWind =
    this->link->GetWorldPose().rot.Ign().Inverse().RotateVector(
    this->windDirection*velocity);
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

  // Moving the previous time and velocity one step forward.
  this->previousVarVel = this->currentVarVel;
  this->previousTime = this->currentTime;
}

GZ_REGISTER_MODEL_PLUGIN(UsvWindPlugin);
