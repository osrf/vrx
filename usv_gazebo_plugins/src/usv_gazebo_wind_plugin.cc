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

#include <std_msgs/Float64.h>
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
  this->world = _parent->GetWorld();

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
    double windAngle = _sdf->GetElement("wind_direction")->Get<double>();
    this->windDirection[0] = cos(windAngle * M_PI / 180);
    this->windDirection[1] = sin(windAngle * M_PI / 180);
    this->windDirection[3] = 0;
  }

  gzmsg << "Wind direction unit vector = " << this->windDirection << std::endl;

  if (_sdf->HasElement("wind_coeff_vector"))
  {
    windCoeff =
      _sdf->GetElement("wind_coeff_vector")->Get<ignition::math::Vector3d>();
  }

  gzmsg << "Wind coefficient vector = " << this->windCoeff << std::endl;

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

  if (_sdf->HasElement("publishing_buffer"))
  {
    this->publishingBuffer =
      _sdf->GetElement("publishing_buffer")->Get<double>();
  }

  gzmsg << "publishing buffer  = " << this->publishingBuffer << std::endl;

  // setting seed for ignition::math::Rand
  if (_sdf->HasElement("random_seed") &&
    _sdf->GetElement("random_seed")->Get<int>() != 0)
  {
    ignition::math::Rand::Seed(
      _sdf->GetElement("random_seed")->Get<int>());
  }
  else{
    common::Time currentWallTime;
    currentWallTime.SetToWallTime();
    ignition::math::Rand::Seed(currentWallTime.sec);
  }

  gzmsg << "Random seed value = " << this->timeConstant << std::endl;

  // initialize previous time and previous velocity
#if GAZEBO_MAJOR_VERSION >= 8
  this->previousTime = this->world->SimTime().Double();
#else
  this->previousTime = this->world->GetSimTime().Double();
#endif
  this->previousVarVel = 0;

  // Initialize ROS transport.
  this->rosNode.reset(new ros::NodeHandle());
  this->windSpeedPub =
      this->rosNode->advertise<std_msgs::Float64>(this->topicWindSpeed, 100);
  this->windDirectionPub = this->rosNode->advertise<std_msgs::Float64>(
      this->topicWindDirection, 100);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&UsvWindPlugin::Update, this));
}

//////////////////////////////////////////////////
void UsvWindPlugin::Update()
{

#if GAZEBO_MAJOR_VERSION >= 8
  double currentTime = this->world->SimTime().Double();
#else
  double currentTime = this->world->GetSimTime().Double();
#endif

  double dT= currentTime - this->previousTime;
  double randomDist = ignition::math::Rand::DblNormal(0, 1);
  // calculate current variable wind velocity
  double currentVarVel = this->previousVarVel + (-1/this->timeConstant*
    (this->previousVarVel+this->gainConstant*randomDist))*dT;
  // calculate current wind velocity
  double velocity = currentVarVel + this->windMeanVelocity;

  // Transform wind from world coordinates to body coordinates
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Vector3d relativeWind =
    this->link->WorldPose().Rot().Inverse().RotateVector(
      this->windDirection*velocity);
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
  this->previousVarVel = currentVarVel;
  this->previousTime = currentTime;

  // Publishing the wind speed and direction
  if (currentTime - this->lastPublishTime > this->publishingBuffer){
    std_msgs::Float64 windSpeedMsg;
    std_msgs::Float64 windDirectionMsg;
    windSpeedMsg.data = velocity;
    windDirectionMsg.data =
        atan2(this->windDirection[1], this->windDirection[0]) * 180 / M_PI;
    this->windSpeedPub.publish(windSpeedMsg);
    this->windDirectionPub.publish(windDirectionMsg);
    this->lastPublishTime = currentTime;
  }
}

GZ_REGISTER_MODEL_PLUGIN(UsvWindPlugin);
