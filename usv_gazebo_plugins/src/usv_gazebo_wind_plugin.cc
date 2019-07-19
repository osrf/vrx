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
void UsvWindPlugin::LinkCoeff::CalculateWindCoeff()
{
  if (this->linkName != "" && this->link)
  {
    double r = 1.2;
    double cd = 0;
    // add together all the wind coeffs of all collision links in the
    // frame of the parent link
    this->windCoeff = ignition::math::Vector3d::Zero;
    for (auto& collision : this->link->GetCollisions())
    {
      ignition::math::Vector3d windCoeff = ignition::math::Vector3d::Zero;
      sdf::ElementPtr collisionSDF = collision->GetSDF();
      if (!collisionSDF->HasElement("geometry") ||
          !collisionSDF->GetElement("geometry"))
        return;
      sdf::ElementPtr geometry = collisionSDF->GetElement("geometry");
      // if box, approximate the the cd as a cube = 0.8
      if (geometry->HasElement("box"))
      {
        ignition::math::Vector3d size = geometry->GetElement("box")-> 
          GetElement("size")->Get<ignition::math::Vector3d>();
	cd = 0.8;
	windCoeff.X(size.Y()*size.Z()*cd*r/(2.0));
	windCoeff.Y(size.X()*size.Z()*cd*r/(2.0));
	windCoeff.Z(size.X()*size.Y()*cd*r/(2.0)); 
      }
      // Transform windCoeff from collision coordinates to link coordinates
      ignition::math::Vector3d temp = collision->
        InitialRelativePose().Rot().Inverse().RotateVector(windCoeff);

      this->windCoeff.X() = temp.X();
      this->windCoeff.Y() = temp.Y();

      this->windCoeff.Z(
        this->windCoeff.Z() +
        (this->windCoeff.Y()*collision->
        InitialRelativePose().Pos().X()) +
        (this->windCoeff.X()*collision->
        InitialRelativePose().Pos().Y())
	);
    }
  }
}

void UsvWindPlugin::WindObj::PopulateLinks()
{
  for (auto& link : this->model->GetLinks())
  {
    this->lcs.push_back(UsvWindPlugin::LinkCoeff());
    this->lcs.back().link = link;
    this->lcs.back().linkName = link->GetName();
    this->lcs.back().CalculateWindCoeff();
    
    gzdbg << link->GetName() << std::endl;
    gzdbg << this->lcs.back().windCoeff.X() << std::endl;
    gzdbg << this->lcs.back().windCoeff.Y() << std::endl;
    gzdbg << this->lcs.back().windCoeff.Z() << std::endl;
  }
}

//////////////////////////////////////////////////
UsvWindPlugin::UsvWindPlugin()
{
}

//////////////////////////////////////////////////
void UsvWindPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  if (char* env_dbg = std::getenv("VRX_DEBUG"))
  {
    gzdbg << std::string(env_dbg) << std::endl;
    if (std::string(env_dbg) == "false")
      this->debug = false;
  }
  else
  {
    gzwarn << "VRX_DEBUG environment variable not set, defaulting to true"
      << std::endl;
  }
  this->world = _parent;
  // Retrieve models' parameters from SDF
  if (!_sdf->HasElement("wind_obj") ||
      !_sdf->GetElement("wind_obj"))
  {
    gzerr << "Did not find SDF parameter wind_obj" << std::endl;
  }
  else
  {
    sdf::ElementPtr windObjSDF = _sdf->GetElement("wind_obj");
    while (windObjSDF)
    {
      UsvWindPlugin::WindObj obj;
      if (!windObjSDF->HasElement("model_name") ||
          !windObjSDF->GetElement("model_name")->GetValue())
      {
        gzerr << ("Did not find SDF parameter model_name") << std::endl;
      }
      else
      {
        obj.modelName = windObjSDF->GetElement("model_name")->Get<std::string>();
        obj.model = this->world->ModelByName(obj.modelName);
      }
      // the links which the wind force will be applied to
      // if no links listed, apply to all links
      if (!windObjSDF->HasElement("link") ||
          !windObjSDF->GetElement("link")->GetValue())
      {
        gzmsg << "Applying forces of wind on all links of" << obj.modelName
          << " automatically." << std::endl;
	obj.PopulateLinks();
      }
      else
      {
        sdf::ElementPtr linkSDF = _sdf->GetElement("link");
        while (linkSDF)
	{
	  
          if (!linkSDF->HasElement("link_name") ||
              !linkSDF->GetElement("link_name")->GetValue())
          gzerr << "link_name required" << std::endl;
	  
          obj.lcs.push_back(UsvWindPlugin::LinkCoeff());

          obj.lcs.back().linkName = linkSDF->GetElement("link_name")->Get<std::string>();
          obj.lcs.back().link = obj.model->GetLink(obj.lcs.back().linkName);

         if (!linkSDF->HasElement("coeff_vector") ||
             !linkSDF->GetElement("coeff_vector")->GetValue())
         {
           gzmsg << "Calculating the wind coefficients of " << obj.lcs.back().linkName <<
            " automatically" << std::endl;
           obj.lcs.back().CalculateWindCoeff();
         }
         else
         {
           obj.lcs.back().windCoeff = windObjSDF->GetElement("coeff_vector")->
             Get<ignition::math::Vector3d>();
         }
	 linkSDF = linkSDF->GetNextElement("link");
        }
      }
      this->windObjs.push_back(obj);
      gzdbg << obj.modelName << " loaded" << std::endl;
      windObjSDF = windObjSDF->GetNextElement("wind_obj");
    }
  }

  if (_sdf->HasElement("wind_direction"))
  {
    double windAngle = _sdf->GetElement("wind_direction")->Get<double>();
    this->windDirection.X(cos(windAngle * M_PI / 180));
    this->windDirection.Y(sin(windAngle * M_PI / 180));
    this->windDirection.Z(0);
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

  if (_sdf->HasElement("update_rate"))
  {
    this->updateRate =
      _sdf->GetElement("update_rate")->Get<double>();
  }

  gzmsg << "update rate  = " << this->updateRate << std::endl;

  // setting seed for ignition::math::Rand
  if (_sdf->HasElement("random_seed") &&
    _sdf->GetElement("random_seed")->Get<int>() != 0)
  {
    ignition::math::Rand::Seed(
      _sdf->GetElement("random_seed")->Get<int>());
  }
  else
  {
    common::Time currentWallTime;
    currentWallTime.SetToWallTime();
    ignition::math::Rand::Seed(currentWallTime.sec);
  }

  gzmsg << "Random seed value = " << this->timeConstant << std::endl;

  // Calculate filter constant
  this->filterGain = this->gainConstant*sqrt(2.0*this->timeConstant);
  gzmsg << "Var wind filter gain = " << this->filterGain << std::endl;

  // Initialize previous time and previous velocity
#if GAZEBO_MAJOR_VERSION >= 8
  this->previousTime = this->world->SimTime().Double();
#else
  this->previousTime = this->world->GetSimTime().Double();
#endif
  this->varVel = 0;
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
  // Look for the missing models if not all of them have been initialized
  if (!this->windObjsInit)
  {
    for (auto& i : this->windObjs)
    {
#if GAZEBO_MAJOR_VERSION >= 8
      if ((!i.init) && (this->world->ModelByName(i.modelName)))
      {
#else
      if ((!i.init) && (this->world->GetModel(i.modelName)))
      {
#endif
        // gzdbg << i.modelName << " initialized" << std::endl;
        ++this->windObjsInitCount;
        i.init = true;
#if GAZEBO_MAJOR_VERSION >= 8
        i.model = this->world->ModelByName(i.modelName);
#else
        i.model = this->world->GetModel(i.modelName);
#endif
      }
    }
    if (windObjsInitCount == windObjs.size())
      this->windObjsInit = true;
  }
#if GAZEBO_MAJOR_VERSION >= 8
  double currentTime = this->world->SimTime().Double();
#else
  double currentTime = this->world->GetSimTime().Double();
#endif
  double dT= currentTime - this->previousTime;
  double randomDist = ignition::math::Rand::DblNormal(0, 1);
  // Current variable wind velocity
  this->varVel += 1.0/this->timeConstant*
    (-1.0*this->varVel+this->filterGain/sqrt(dT)*randomDist)*dT;
  // Current wind velocity
  double velocity = this->varVel + this->windMeanVelocity;

  for (auto& windObj : this->windObjs)
  {
    // Apply the forces of the wind to all wind objects only if they have been
    // initialized
    if (!windObj.init)
      continue;
    for (auto& lc : windObj.lcs)
    {
      if (!lc.link)
        continue;
      // Transform wind from world coordinates to body coordinates
#if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Vector3d relativeWind =
        lc.link->WorldPose().Rot().Inverse().RotateVector(
        this->windDirection*velocity);
#else
      ignition::math::Vector3d relativeWind =
        lc.link->GetWorldPose().rot.Ign().Inverse().RotateVector(
        this->windDirection*velocity);
#endif
      // Calculate apparent wind
#if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Vector3d apparentWind =
        relativeWind - lc.link->RelativeLinearVel();
#else
      ignition::math::Vector3d apparentWind = relativeWind
        - lc.link->GetRelativeLinearVel().Ign();
#endif

      // gzdbg << "Relative wind: " << relativeWind << std::endl;
      // gzdbg << "Apparent wind: " << apparentWind << std::endl;

      // Calculate wind force - body coordinates
      ignition::math::Vector3d windForce(
        lc.windCoeff.X() * relativeWind.X() * abs(relativeWind.X()),
        lc.windCoeff.Y() * relativeWind.Y() * abs(relativeWind.Y()),
        -2.0 * lc.windCoeff.Z() * relativeWind.X() * relativeWind.Y());

      // Add forces/torques to link at CG
      lc.link->AddRelativeForce(
        ignition::math::Vector3d(windForce.X(), windForce.Y(), 0.0));
      lc.link->AddRelativeTorque(
        ignition::math::Vector3d(0.0, 0.0, windForce.Z()));
    }
  }
  // Moving the previous time one step forward.
  this->previousTime = currentTime;

  double publishingBuffer = 1/this->updateRate;
  if (this->updateRate >= 0)
  {
    publishingBuffer = 1/this->updateRate;
  }
  else
  {
    publishingBuffer = -1;
  }
  // Publishing the wind speed and direction
  if ((currentTime - this->lastPublishTime > publishingBuffer) && this->debug)
  {
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

GZ_REGISTER_WORLD_PLUGIN(UsvWindPlugin);
