/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include "usv_gazebo_plugins/buoyancy_gazebo_plugin.hh"

using namespace gazebo;

/////////////////////////////////////////////////
BuoyancyPlugin::BuoyancyPlugin()
  : fluidDensity(999.1026),
    fluidLevel(0.0),
    fluidDrag(0.0)
{
}

/////////////////////////////////////////////////
void BuoyancyPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != NULL, "Received NULL model pointer");
  GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer");

  if (_sdf->HasElement("fluid_density"))
  {
    this->fluidDensity = _sdf->Get<double>("fluid_density");
  }
  if (_sdf->HasElement("fluid_level"))
  {
    this->fluidLevel = _sdf->Get<double>("fluid_level");
  }
  if (_sdf->HasElement("fluid_drag"))
  {
    this->fluidDrag = _sdf->Get<double>("fluid_drag");
  }

  if (_sdf->HasElement("link"))
  {
    gzmsg << "Found that SDF has at least one link element, looking at each "
          << "link..." << std::endl;
    int counter = 0;
    for (sdf::ElementPtr linkElem = _sdf->GetElement("link"); linkElem;
         linkElem = linkElem->GetNextElement("link"))
    {
      // Print each attribute for the link
      gzmsg << "Looking for name attribute in link number " << counter
            << ", which has " << linkElem->GetAttributeCount()
            << " attributes" << std::endl;
      counter++;
      int id = -1;
      std::string name = "";

      if (linkElem->HasElement("name"))
      {
        name = linkElem->GetElement("name")->Get<std::string>();
        gzmsg << "Found link name in SDF [" << name << "]" << std::endl;
        physics::LinkPtr link = _model->GetLink(name);
        if (!link)
        {
          gzwarn << "Specified link [" << name << "] not found." << std::endl;
          continue;
        }
        id = link->GetId();
        // Add this link to our list for applying buoy forces
        this->buoyancyLinks.push_back(link);
      }
      else
      {
        gzwarn << "Missing 'name' element within link number ["
               << counter - 1 << "] in SDF" << std::endl;
        // Exit if we didn't set ID
        continue;
      }

      if (this->volPropsMap.count(id) != 0)
      {
        gzwarn << "Properties for link [" << name << "] already set, skipping "
               << "second property block" << std::endl;
        continue;
      }

      if (linkElem->HasElement("center_of_volume"))
      {
        math::Vector3 cov = linkElem->GetElement("center_of_volume")
            ->Get<math::Vector3>();
        this->volPropsMap[id].cov = cov;
      }
      else
      {
        gzwarn << "Required element center_of_volume missing from link ["
               << name << "] in BuoyancyPlugin SDF" << std::endl;
        continue;
      }

      if (linkElem->HasElement("area"))
      {
        double area = linkElem->GetElement("area")->Get<double>();
        if (area <= 0)
        {
          gzwarn << "Nonpositive area specified in BuoyancyPlugin!"
                 << std::endl;
          // Remove the element from the map since it's invalid.
          this->volPropsMap.erase(id);
          continue;
        }
        this->volPropsMap[id].area = area;
      }
      else
      {
        gzwarn << "Required element 'area' missing from element link [" << name
               << "] in BuoyancyPlugin SDF" << std::endl;
        continue;
      }

      if (linkElem->HasElement("height"))
      {
        double height = linkElem->GetElement("height")->Get<double>();
        if (height <= 0)
        {
          gzwarn << "Nonpositive height specified in BuoyancyPlugin!"
                 << std::endl;
          // Remove the element from the map since it's invalid.
          this->volPropsMap.erase(id);
          continue;
        }
        this->volPropsMap[id].height = height;
      }
      else
      {
        gzwarn << "Required element 'height' missing from element link ["
               << name << "] in BuoyancyPlugin SDF" << std::endl;
        continue;
      }
    }
  }
}

/////////////////////////////////////////////////
void BuoyancyPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&BuoyancyPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void BuoyancyPlugin::OnUpdate()
{
  for (auto &link : this->buoyancyLinks)
  {
    VolumeProperties volumeProperties = this->volPropsMap[link->GetId()];
    double height = volumeProperties.height;
    double area = volumeProperties.area;
    double volume = height * area;
    math::Pose linkFrame = link->GetWorldPose();

    // Location of bottom of object relative to the fluid surface - assumes
    // origin is at cog of the object.
    double bottomRelSurf = this->fluidLevel - (linkFrame.pos.z - height / 2.0);

    // out of water
    if (bottomRelSurf <= 0)
    {
      volume = 0.0;
    }
    // at surface
    else if (bottomRelSurf <= height)
    {
      volume = bottomRelSurf * area;
    }
    else
    {
      volume = height * area;
    }

    GZ_ASSERT(volume >= 0, "Nonpositive volume found in volume properties!");

    // By Archimedes' principle,
    // buoyancy = -(mass*gravity)*fluid_density/object_density
    // object_density = mass/volume, so the mass term cancels.
    // Therefore,
    //    math::Vector3 buoyancy =
    //    -this->fluidDensity * volume * this->model->GetWorld()->Gravity();
    const math::Vector3 kGravity(0, 0, -9.81);
    math::Vector3 buoyancy = -this->fluidDensity * volume * kGravity;

    // Add some drag
    if (volume > 1e-6)
    {
      math::Vector3 vel = link->GetWorldLinearVel();
      double dz = -1.0 * this->fluidDrag * vel.z * std::abs(vel.z);
      math::Vector3 drag(0, 0, dz);
      buoyancy += drag;
      if (buoyancy.z < 0.0)
      {
        buoyancy.z = 0.0;
      }
    }

    // rotate buoyancy into the link frame before applying the force.
    math::Vector3 buoyancyLinkFrame =
      linkFrame.rot.GetInverse().RotateVector(buoyancy);

    link->AddForceAtRelativePosition(buoyancy, volumeProperties.cov);
  }
}

GZ_REGISTER_MODEL_PLUGIN(BuoyancyPlugin)
