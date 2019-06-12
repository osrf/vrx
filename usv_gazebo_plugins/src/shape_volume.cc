#include "usv_gazebo_plugins/shape_volume.hh"

using namespace buoyancy;

/////////////////////////////////////////////////
ShapeVolumePtr ShapeVolume::makeShape(const sdf::ElementPtr sdf)
{
  double epsilon = 1e-20;

  ShapeVolume* shape = nullptr;

  if (sdf->HasElement("box"))
  {
    auto boxElem = sdf->GetElement("box");
    if (boxElem->HasElement("size"))
    {
      ignition::math::Vector3d dim = boxElem->GetElement("size")
          ->Get<ignition::math::Vector3d>();
      if (dim[0] > epsilon && dim[1] > epsilon && dim[2] > epsilon)
      {
        shape = dynamic_cast<ShapeVolume*>(new BoxVolume(dim[0], dim[1], dim[2]));
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
  else if (sdf->HasElement("cylinder"))
  {
    auto cylinderElem = sdf->GetElement("cylinder");
    if (cylinderElem->HasElement("radius") && cylinderElem->HasElement("length"))
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
  } else {
    throw ParseException("geometry", "missing <box>, <cylinder> or <sphere> element");
  }

  return std::unique_ptr<ShapeVolume>(shape);
}

/////////////////////////////////////////////////
std::string ShapeVolume::display()
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
BoxVolume::BoxVolume(double x, double y, double z)
    : x(x),
      y(y),
      z(z),
      polyhedron_(Polyhedron::makeCube(x, y, z))
{
  type = ShapeType::Box;
}

//////////////////////////////////////////////////
std::string BoxVolume::display()
{
  std::stringstream ss;
  ss << ShapeVolume::display() << ":" << x << "," << y << "," << z;
  return ss.str();
}

//////////////////////////////////////////////////
Volume BoxVolume::calculateVolume(const ignition::math::Pose3d &pose,
                                 double fluidLevel)
{
  Plane waterSurface;
  waterSurface.offset = fluidLevel;
  return polyhedron_.submergedVolume(pose.Pos(), pose.Rot(), waterSurface);
}

/////////////////////////////////////////////////
CylinderVolume::CylinderVolume(double r, double h)
    : r(r),
      h(h),
      polyhedron_(Polyhedron::makeCylinder(r, h, 20))
{
  type = ShapeType::Cylinder;
}

/////////////////////////////////////////////////
std::string CylinderVolume::display()
{
  std::stringstream ss;
  ss << ShapeVolume::display() << ":" << r << "," << h;
  return ss.str();
}

/////////////////////////////////////////////////
Volume CylinderVolume::calculateVolume(const ignition::math::Pose3d &pose,
                                      double fluidLevel)
{
  Plane waterSurface;
  waterSurface.offset = fluidLevel;
  return polyhedron_.submergedVolume(pose.Pos(), pose.Rot(), waterSurface);
}

//////////////////////////////////////////////////
SphereVolume::SphereVolume(double r)
    : r(r)
{
  type = ShapeType::Sphere;
}

//////////////////////////////////////////////////
std::string SphereVolume::display()
{
  std::stringstream ss;
  ss << ShapeVolume::display() << ":" << r;
  return ss.str();
}

//////////////////////////////////////////////////
Volume SphereVolume::calculateVolume(const ignition::math::Pose3d &pose,
                                    double fluidLevel)
{
  Volume output{};
  // Location of bottom of object relative to the fluid surface - assumes
  // origin is at cog of the object.
  double h = fluidLevel - (pose.Pos().Z() - r);
  // out of water
  if (h <= 0)
  {
    return output;
  }
    // at surface
  else if (h <= 2 * r)
  {
    output.volume = (h * h *  r - ::pow(h, 3) / 3.0) * M_PI;
  }
  else
  {
    output.volume = 4. / 3. * M_PI * ::pow(r, 3);
  }
  return output;
}