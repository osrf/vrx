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

#include "ShapeVolume.hh"

using namespace gz;
using namespace vrx;

/////////////////////////////////////////////////
ShapeVolumePtr ShapeVolume::makeShape(const sdf::ElementPtr _sdf)
{
  double epsilon = 1e-20;

  ShapeVolume* shape = nullptr;

  if (_sdf->HasElement("box"))
  {
    auto boxElem = _sdf->GetElement("box");
    if (boxElem->HasElement("size"))
    {
      math::Vector3d dim = boxElem->GetElement("size")->Get<math::Vector3d>();
      if (dim[0] > epsilon && dim[1] > epsilon && dim[2] > epsilon)
      {
        shape = dynamic_cast<ShapeVolume*>(
            new BoxVolume(dim[0], dim[1], dim[2]));
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
  else if (_sdf->HasElement("sphere"))
  {
    auto sphereElem = _sdf->GetElement("sphere");
    if (sphereElem->HasElement("radius"))
    {
      auto r = sphereElem->GetElement("radius")->Get<double>();
      if (r > epsilon)
      {
        shape = dynamic_cast<ShapeVolume*>(new SphereVolume(r));
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
  else if (_sdf->HasElement("cylinder"))
  {
    auto cylinderElem = _sdf->GetElement("cylinder");
    if (cylinderElem->HasElement("radius") &&
        cylinderElem->HasElement("length"))
    {
      auto r = cylinderElem->GetElement("radius")->Get<double>();
      auto l = cylinderElem->GetElement("length")->Get<double>();
      if (r > epsilon || l > epsilon)
      {
        shape = dynamic_cast<ShapeVolume*>(new CylinderVolume(r, l));
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
  }
  else
  {
    throw ParseException(
        "geometry", "missing <box>, <cylinder> or <sphere> element");
  }

  return std::unique_ptr<ShapeVolume>(shape);
}

/////////////////////////////////////////////////
std::string ShapeVolume::Display() const
{
  switch (this->type)
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
  return "None";
}

//////////////////////////////////////////////////
BoxVolume::BoxVolume(double _x, double _y, double _z)
    : x(_x),
      y(_y),
      z(_z),
      polyhedron(Polyhedron::makeCube(_x, _y, _z))
{
  this->type = ShapeType::Box;
  this->volume = _x * _y * _z;
  this->averageLength = (_x + _y + _z) / 3.0;
}

//////////////////////////////////////////////////
std::string BoxVolume::Display() const
{
  std::stringstream ss;
  ss << ShapeVolume::Display() << ":"
     << this->x << "," << this->y << "," << this->z;
  return ss.str();
}

//////////////////////////////////////////////////
Volume BoxVolume::CalculateVolume(const math::Pose3d &_pose,
                                  double _fluidLevel) const
{
  Plane waterSurface;
  waterSurface.offset = _fluidLevel;
  return this->polyhedron.SubmergedVolume(
    _pose.Pos(), _pose.Rot(), waterSurface);
}

/////////////////////////////////////////////////
CylinderVolume::CylinderVolume(double _r, double _h)
    : r(_r),
      h(_h),
      polyhedron(Polyhedron::makeCylinder(_r, _h, 20))
{
  this->type = ShapeType::Cylinder;
  this->volume = M_PI * _r * _r * _h;
  this->averageLength = (2 * _r + _h) / 2.0;
}

/////////////////////////////////////////////////
std::string CylinderVolume::Display() const
{
  std::stringstream ss;
  ss << ShapeVolume::Display() << ":" << this->r << "," << this->h;
  return ss.str();
}

/////////////////////////////////////////////////
Volume CylinderVolume::CalculateVolume(const math::Pose3d &_pose,
                                       double _fluidLevel) const
{
  Plane waterSurface;
  waterSurface.offset = _fluidLevel;
  return this->polyhedron.SubmergedVolume(
    _pose.Pos(), _pose.Rot(), waterSurface);
}

//////////////////////////////////////////////////
SphereVolume::SphereVolume(double _r)
    : r(_r)
{
  this->type = ShapeType::Sphere;
  this->volume = 4./3. * M_PI * _r * _r * _r;
  this->averageLength = 2 * _r;
}

//////////////////////////////////////////////////
std::string SphereVolume::Display() const
{
  std::stringstream ss;
  ss << ShapeVolume::Display() << ":" << this->r;
  return ss.str();
}

//////////////////////////////////////////////////
Volume SphereVolume::CalculateVolume(const math::Pose3d &_pose,
                                     double _fluidLevel) const
{
  Volume output{};
  // Location of bottom of object relative to the fluid surface - assumes
  // origin is at cog of the object.
  double h = _fluidLevel - (_pose.Pos().Z() - r);

  if (h <= 0)
    return output;

  // limits of integration
  double intLimitLower = -r;
  double intLimitUpper = (-r + h) > r ? r : (-r + h);

  // volume = integral of (R^2 - z^2) dz * pi
  output.volume = (pow(r, 2) * (intLimitUpper - intLimitLower)
      - (pow(intLimitUpper, 3) / 3.0 - pow(intLimitLower, 3) / 3.0)) * M_PI;
  output.centroid.X() = _pose.Pos().X();
  output.centroid.Y() = _pose.Pos().Y();

  if (output.volume > 0)
  {
    // centroid is always centered to object in X and Y plane
    output.centroid.X() = _pose.Pos().X();
    output.centroid.Y() = _pose.Pos().Y();
    // z_bar = (integral of (z(R)^2 - z^3) dz) * pi / volume
    output.centroid.Z() = (pow(r, 2) / 2.0 *
        (pow(intLimitUpper, 2) - pow(intLimitLower, 2)) -
        (pow(intLimitUpper, 4) - pow(intLimitLower, 4))/ 4.0)
            * M_PI / output.volume;
    // convert centroid.z to global frame
    output.centroid.Z() = _pose.Pos().Z() + output.centroid.Z();
  }
  return output;
}
