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
#include <ignition/math/Pose3.hh>
#include "usv_gazebo_plugins/buoyancy_gazebo_plugin.hh"

using namespace gazebo;
using namespace gazebo::buoyancy;

/////////////////////////////////////////////////
ShapePtr Shape::makeShape(const sdf::ElementPtr sdf)
{
  double epsilon = 1e-20;

  Shape* shape = nullptr;

  if (sdf->HasElement("box"))
  {
    auto boxElem = sdf->GetElement("box");
    if (boxElem->HasElement("size"))
    {
      ignition::math::Vector3d dim = boxElem->GetElement("size")
          ->Get<ignition::math::Vector3d>();
      if (dim[0] > epsilon && dim[1] > epsilon && dim[2] > epsilon)
      {
        shape = dynamic_cast<Shape*>(new BoxShape(dim[0], dim[1], dim[2]));
      }
      else
      {
        throw ParseException("box", "incorrect dimensions");
      }
    }
    else
    {
      throw ParseException("box", "missing <size> element");
    }
  }
  else if (sdf->HasElement("sphere"))
  {
    auto sphereElem = sdf->GetElement("sphere");
    if (sphereElem->HasElement("radius"))
    {
      auto r = sphereElem->GetElement("radius")->Get<double>();
      if (r > epsilon)
      {
        shape = dynamic_cast<Shape*>(new SphereShape(r));
      }
      else
      {
        throw ParseException("sphere", "incorrect dimensions");
      }
    }
    else
    {
      throw ParseException("sphere", "missing <radius> element");
    }
  }
  else if (sdf->HasElement("cylinder"))
  {
    auto cylinderElem = sdf->GetElement("cylinder");
    if (cylinderElem->HasElement("radius") && cylinderElem->HasElement("length"))
    {
      auto r = cylinderElem->GetElement("radius")->Get<double>();
      auto l = cylinderElem->GetElement("length")->Get<double>();
      if (r > epsilon || l > epsilon)
      {
        shape = dynamic_cast<Shape*>(new CylinderShape(r, l));
      }
      else
      {
        throw ParseException("cylinder", "incorrect dimensions");
      }
    }
    else
    {
      throw ParseException("cylinder", "missing <radius> or <length> element");
    }
  } else {
    throw ParseException("geometry", "missing <box>, <cylinder> or <sphere> element");
  }

  return std::unique_ptr<Shape>(shape);
}

/////////////////////////////////////////////////
std::string Shape::disp()
{
  switch(type)
  {
    case ShapeType::None:
      return "None";
    case ShapeType::Box:
      return "Box";
    case ShapeType::Cylinder:
      return "Cylinder";
    case ShapeType::Sphere:
      return "Sphere";
  }
}

//////////////////////////////////////////////////
BoxShape::BoxShape(double x, double y, double z)
  : x(x),
    y(y),
    z(z)
{
  type = ShapeType::Box;
}

//////////////////////////////////////////////////
std::string BoxShape::disp()
{
  std::stringstream ss;
  ss << Shape::disp() << ":" << x << "," << y << "," << z;
  return ss.str();
}

/////////////////////////////////////////////////
CylinderShape::CylinderShape(double r, double h)
  : r(r),
    h(h)
{
  type = ShapeType::Cylinder;
}

/////////////////////////////////////////////////
std::string CylinderShape::disp()
{
  std::stringstream ss;
  ss << Shape::disp() << ":" << r << "," << h;
  return ss.str();
}

//////////////////////////////////////////////////
SphereShape::SphereShape(double r)
  : r(r)
{
}

//////////////////////////////////////////////////
std::string SphereShape::disp()
{
  std::stringstream ss;
  ss << Shape::disp() << ":" << r;
  return ss.str();
}

//////////////////////////////////////////////////
BuoyancyObject::BuoyancyObject()
  : linkId(-1),
    linkName(""),
    pose(0, 0, 0, 0, 0, 0),
    shape(nullptr)
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
      shape = Shape::makeShape(geometry);
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
      << "\tgeometry " << shape->disp();
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
        gzmsg << buoyObj.disp() << std::endl;

        // add link to linkMap if it is not in the map
        if (linkMap.find(buoyObj.linkId) == linkMap.end())
        {
          linkMap[buoyObj.linkId] = _model->GetLink(buoyObj.linkName);
        }
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
  for (auto &link : this->buoyancyLinks)
  {
    VolumeProperties volumeProperties = this->volPropsMap[link->GetId()];
    double height = volumeProperties.height;
    double area = volumeProperties.area;
    double volume = height * area;
    #if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Pose3d linkFrame = link->WorldPose();
    #else
      ignition::math::Pose3d linkFrame = link->GetWorldPose().Ign();
    #endif

    // Location of bottom of object relative to the fluid surface - assumes
    // origin is at cog of the object.
    double bottomRelSurf =
      this->fluidLevel - (linkFrame.Pos().Z() - height / 2.0);

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
    const ignition::math::Vector3d kGravity(0, 0, -9.81);
    ignition::math::Vector3d buoyancy = -this->fluidDensity * volume * kGravity;

    // Add some drag
    if (volume > 1e-6)
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

    // rotate buoyancy into the link frame before applying the force.
    ignition::math::Vector3d buoyancyLinkFrame =
      linkFrame.Rot().Inverse().RotateVector(buoyancy);

    link->AddForceAtRelativePosition(buoyancy, volumeProperties.cov);
  }
}

GZ_REGISTER_MODEL_PLUGIN(BuoyancyPlugin)
