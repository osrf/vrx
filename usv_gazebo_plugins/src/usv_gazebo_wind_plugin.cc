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
void UsvWindPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  std::string linkName;
  this->world = _parent;
  // Retrieve models' parameters from SDF
  if (!_sdf->HasElement("models_n") ||
      !_sdf->GetElement("models_n")->GetValue())
  {
    gzerr << "Did not find SDF parameter models_n" << std::endl;
  }
  else
  {
    int n = _sdf->GetElement("models_n")->Get<int>();
    for(int i=0; i<n; ++i)
    {
      std::string model_name ="model_name_" + std::to_string(i);
      if (!_sdf->HasElement(model_name) ||
      !_sdf->GetElement(model_name)->GetValue())
      {
        gzerr << ("Did not find SDF parameter model_name_%d",n) << std::endl;
      }
      else
      {
        std::string link_name ="link_name_" + std::to_string(i);
        if (!_sdf->HasElement(link_name) ||
        !_sdf->GetElement(link_name)->GetValue())
        {
          gzerr << ("Did not find SDF parameter link_name_%d",n) << std::endl;
        }
        else
        {
          std::string coeff_vector ="coeff_vector_" + std::to_string(i);
          if (!_sdf->HasElement(coeff_vector) ||
          !_sdf->GetElement(coeff_vector)->GetValue())
          {
            gzerr << ("Did not find SDF parameter coeff_vector_%d",i) << std::endl;
          }
          else
          { 
	    UsvWindPlugin::WindObj obj; 
	    obj.model_name = _sdf->GetElement(model_name)->Get<std::string>();
	    obj.link_name = _sdf->GetElement(link_name)->Get<std::string>();
	    obj.windCoeff = _sdf->GetElement(coeff_vector)->Get<ignition::math::Vector3d>(); 
	    this->windObjs.push_back(obj);
            gzdbg << obj.model_name << " loaded"<<std::endl;
	  }
        }
      }
    }
  }
  if (_sdf->HasElement("wind_direction"))
  {
    this->windDirection =
      _sdf->GetElement("wind_direction")->Get<ignition::math::Vector3d>();
    this->windDirection = this->windDirection.Normalize();
  }

  gzmsg << "Wind direction unit vector = " << this->windDirection << std::endl;

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

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&UsvWindPlugin::Update, this));
}

//////////////////////////////////////////////////
void UsvWindPlugin::Update()
{
  //look for the missing models if not all of them have been initialized
  if (!this->windObjsInit)
  {
    int objs = 0;
    for (auto& i : this->windObjs)
    {
      if ((!i.init)&&(this->world->ModelByName(i.model_name)))
      {
        gzdbg << i.model_name << " initialized"<<std::endl;
	++objs;
	i.init = true;
        i.model = this->world->ModelByName(i.model_name);
	i.link = i.model->GetLink(i.link_name);
      }
    }
    if(objs == windObjs.size())
      this->windObjsInit = true;
  }
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

  for (auto i : this->windObjs)
  {
    // Apply the forces of the wind to all wind objects only if they have been initialized
    if(i.init)
    { 
      // Transform wind from world coordinates to body coordinates
#if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Vector3d relativeWind =
        i.link->WorldPose().Rot().Inverse().RotateVector(
          this->windDirection*velocity);
#else
      ignition::math::Vector3d relativeWind =
        this->link->GetWorldPose().rot.Ign().Inverse().RotateVector(
        this->windDirection*velocity);
#endif
      // Calculate apparent wind
#if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Vector3d apparentWind =
        relativeWind - i.link->RelativeLinearVel();
#else
      ignition::math::Vector3d apparentWind = relativeWind
        - i.->GetRelativeLinearVel().Ign();
#endif
  
      // gzdbg << "Relative wind: " << relativeWind << std::endl;
      // gzdbg << "Apparent wind: " << apparentWind << std::endl;
   
      // Calculate wind force - body coordinates
      ignition::math::Vector3d windForce(
        i.windCoeff.X() * relativeWind.X() * abs(relativeWind.X()),
        i.windCoeff.Y() * relativeWind.Y() * abs(relativeWind.Y()),
        -2.0 * i.windCoeff.Z() * relativeWind.X() * relativeWind.Y());
   
      // Add forces/torques to link at CG
      i.link->AddRelativeForce(
        ignition::math::Vector3d(windForce.X(), windForce.Y(), 0.0));
      i.link->AddRelativeTorque(
        ignition::math::Vector3d(0.0, 0.0, windForce.Z()));
    }
    // Moving the previous time and velocity one step forward.
    this->previousVarVel = currentVarVel;
    this->previousTime = currentTime;
  }
}

GZ_REGISTER_WORLD_PLUGIN(UsvWindPlugin);
