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
using namespace gazebo::buoyancy;
using namespace ::buoyancy;

//////////////////////////////////////////////////
BuoyancyObject::BuoyancyObject()
  : linkId(-1),
    linkName(""),
    pose(0, 0, 0, 0, 0, 0),
    mass(0.0),
    shape(nullptr)
{
}

///////////////////////////////////////////////////
BuoyancyObject::BuoyancyObject(BuoyancyObject &&obj) noexcept
  : linkId(obj.linkId),
    linkName(obj.linkName),
    pose(obj.pose),
    shape(std::move(obj.shape))
{
}

///////////////////////////////////////////////////
void BuoyancyObject::load(const physics::ModelPtr model,
    const sdf::ElementPtr elem)
{
  // parse link
  if (elem->HasElement("link_name"))
  {
    linkName = elem->GetElement("link_name")->Get<std::string>();
    physics::LinkPtr link = model->GetLink(linkName);
    if (!link)
    {
      throw ParseException("link_name", "invalid link name");
    }
    linkId = link->GetId();
  }
  else
  {
    throw ParseException("link_name", "missing element");
  }

  // parse pose (optional)
  if (elem->HasElement("pose"))
  {
    pose = elem->GetElement("pose")->Get<ignition::math::Pose3d>();
  }

  // parse geometry
  if (elem->HasElement("geometry"))
  {
    sdf::ElementPtr geometry = elem->GetElement("geometry");
    try
    {
      shape = std::move(ShapeVolume::makeShape(geometry));
    }
    catch (...)
    {
      throw;
    }
  }
  else
  {
    throw ParseException("geometry", "missing element");
  }
}

//////////////////////////////////////////////////
std::string BuoyancyObject::disp() {
  std::stringstream ss;
  ss << "Buoyancy object\n"
      << "\tlink: " << linkName << "[" << linkId << "]\n"
      << "\tpose: " << pose << '\n'
      << "\tgeometry " << shape->display() << '\n'
      << "\tmass " << mass;
  return ss.str();
}

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
  GZ_ASSERT(_model != nullptr, "Received NULL model pointer");
  GZ_ASSERT(_sdf != nullptr, "Received NULL SDF pointer");

  model = _model;

  _model->GetLink()->GetInertial()->Mass();

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

  if (_sdf->HasElement("buoyancy"))
  {
    gzmsg << "Found buoyancy element(s), looking at each element..." << std::endl;
    for (sdf::ElementPtr buoyancyElem = _sdf->GetElement("buoyancy"); buoyancyElem;
        buoyancyElem = buoyancyElem->GetNextElement("buoyancy")) {
      try
      {
        BuoyancyObject buoyObj = BuoyancyObject();
        buoyObj.load(_model, buoyancyElem);

        // add link to linkMap if it is not in the map
        if (linkMap.find(buoyObj.linkId) == linkMap.end())
        {
          linkMap[buoyObj.linkId] = _model->GetLink(buoyObj.linkName);
        }
        buoyObj.mass = linkMap[buoyObj.linkId]->GetInertial()->Mass();
        gzmsg << buoyObj.disp() << std::endl;
        // add buoyancy object to list
        buoyancyObjects.push_back(std::move(buoyObj));
      }
      catch (const std::exception& e)
      {
        gzwarn << e.what() << std::endl;
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
  for (auto& buoyancyObj : this->buoyancyObjects)
  {
    auto link = linkMap[buoyancyObj.linkId];
    #if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d linkFrame = link->WorldPose();
    #else
        ignition::math::Pose3d linkFrame = link->GetWorldPose().Ign();
    #endif
    linkFrame = linkFrame * buoyancyObj.pose;

    auto volume = buoyancyObj.shape->calculateVolume(linkFrame, this->fluidLevel);

    GZ_ASSERT(volume.volume >= 0, "Non-positive volume found in volume properties!");

    // By Archimedes' principle,
    // buoyancy = -(mass*gravity)*fluid_density/object_density
    // object_density = mass/volume, so the mass term cancels.
    ignition::math::Vector3d buoyancy = -this->fluidDensity * volume.volume * model->GetWorld()->Gravity();

    // Add some drag
    if (volume.volume > 1e-6)
    {
      #if GAZEBO_MAJOR_VERSION >= 8
            ignition::math::Vector3d vel = link->WorldLinearVel();
      #else
            ignition::math::Vector3d vel= link->GetWorldLinearVel().Ign();
      #endif

      double dz = -1.0 * this->fluidDrag * vel.Z() * std::abs(vel.Z());
      ignition::math::Vector3d drag(0, 0, dz);
      buoyancy += drag;
      if (buoyancy.Z() < 0.0)
      {
        buoyancy.Z() = 0.0;
      }
    }

    // apply force
    link->AddForceAtWorldPosition(buoyancy, volume.centroid);
  }
}

GZ_REGISTER_MODEL_PLUGIN(BuoyancyPlugin)
