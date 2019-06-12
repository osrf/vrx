#include "usv_gazebo_plugins/polyhedron_volume.hpp"

using namespace buoyancy;
using Face = Polyhedron::Face;

//////////////////////////////////////////////////////
Plane::Plane()
  : normal(0., 0., 1.), offset(0.)
{
}

//////////////////////////////////////////////////////
Polyhedron::Face::Face(int _i1, int _i2, int _i3)
  : i1(_i1), i2(_i2), i3(_i3)
{
}

//////////////////////////////////////////////////////
Polyhedron::Volume::Volume()
  : volume(0.0), centroid(Vec3({0,0,0}))
{
}

//////////////////////////////////////////////////////
Polyhedron::Volume&
Polyhedron::Volume::operator+=(const Polyhedron::Volume &rhs)
{
  this->volume += rhs.volume;
  this->centroid += rhs.centroid;
  return *this;
}

//////////////////////////////////////////////////////
Polyhedron Polyhedron::makeCube(double x, double y, double z)
{
  Polyhedron cube;
  // generate vertices
  for(int i=0; i<2; i++)
  {
    for(int j=0; j<2; j++)
    {
      for(int k=0; k<2; k++)
      {
        cube.vertices.emplace_back(Vec3(i * x - x / 2.0, j * y - y / 2.0, k * z - z / 2.0));
      }
    }
  }
  // generate faces
  cube.faces.emplace_back(Face(0,6,4));
  cube.faces.emplace_back(Face(0,2,6));
  cube.faces.emplace_back(Face(0,3,2));
  cube.faces.emplace_back(Face(0,1,3));
  cube.faces.emplace_back(Face(2,7,6));
  cube.faces.emplace_back(Face(2,3,7));
  cube.faces.emplace_back(Face(4,6,7));
  cube.faces.emplace_back(Face(4,7,5));
  cube.faces.emplace_back(Face(0,4,5));
  cube.faces.emplace_back(Face(0,5,1));
  cube.faces.emplace_back(Face(1,5,7));
  cube.faces.emplace_back(Face(1,7,3));

  return cube;
}

//////////////////////////////////////////////////////
Polyhedron Polyhedron::makeCylinder(double r, double l, int n)
{
  assert(n > 4);

  buoyancy::Polyhedron cylinder;

  // generate all vertices
  double angle_step = 2.0 * M_PI / n;
  double l_2 = l / 2.0;
  cylinder.vertices.resize(2*n+2);
  cylinder.vertices[0] = Vec3{0,0,-l_2};
  for(int i=1; i<=n; i++)
  {
    double x = r * ::sin(angle_step * (i-1));
    double y = r * ::cos(angle_step * (i-1));
    // bottom plate
    cylinder.vertices[i] = Vec3{x, y, -l_2};
    // top plate
    cylinder.vertices[i+n] = Vec3{x, y, l_2};
  }
  cylinder.vertices[2*n+1] = Vec3{0,0,l_2};

  // generate all faces
  for(int i=1; i<=n; i++)
  { // bottom plate
    cylinder.faces.emplace_back(Face(0, i, (i%n)+1));
  }
  for(int i=1; i<=n; i++)
  { // walls
    cylinder.faces.emplace_back(Face(i+1, i, n+i));
    cylinder.faces.emplace_back(Face((i%n)+n, (i%n)+n+1, (i%n)+1));
  }
  for(int i=1; i<=n; i++)
  { // top plate
    cylinder.faces.emplace_back(Face(i+n, 2*n+1, (i%n)+n+1));
  }

  assert(cylinder.vertices.size() == 2 * n + 2);
  assert(cylinder.faces.size() == 4 * n);

  return cylinder;
}

//////////////////////////////////////////////////////
Polyhedron::Volume Polyhedron::tetrahedronVolume(const Vec3 &v1,
    const Vec3 &v2, const Vec3 &v3, const Vec3 &p)
{
  Vec3 a = v2 - v1;
  Vec3 b = v3 - v1;
  Vec3 r = p - v1;

  Volume output;
  output.volume = (1/6.) * (b.Cross(a)).Dot(r);
  output.centroid = 0.25 * output.volume * (v1 + v2 + v3 + p);
  return output;
}

//////////////////////////////////////////////////////
Polyhedron::Volume Polyhedron::computeFullVolume()
{
  Volume output;
  // Compute the contribution of each triangle face
  for(const auto& face : faces)
  {
    Vec3 v1 = vertices[face.i1];
    Vec3 v2 = vertices[face.i2];
    Vec3 v3 = vertices[face.i3];
    output += tetrahedronVolume(v1, v2, v3);
  }
  return output;
}

//////////////////////////////////////////////////////
Polyhedron::Volume
Polyhedron::clipTriangle(const Vec3 &v1, const Vec3 &v2,
    const Vec3 &v3, double d1, double d2, double d3, const Vec3 &p)
{
  assert(d1 * d2 < 0);
  Volume output;

  // calculate the intersection point from a to b
  Vec3 ab = v1 + (d1/(d1 - d2))*(v2 - v1);
  if (d1 < 0)
  {
    // b to c crosses the clipping plane
    if (d3 < 0)
    {
      // Case B - a quadrilateral or two triangles
      // Calculate intersection point from b to c.
      Vec3 bc = v2 + (d2/(d2 - d3))*(v3 - v2);
      output += tetrahedronVolume(ab, bc, v1, p);
      output += tetrahedronVolume(bc, v3, v1, p);
    }
    else
    {
      // Case A - a single triangle.
      Vec3 ac = v1 + (d1/(d1 - d3))*(v3 - v1);
      output += tetrahedronVolume(ab, ac, v1, p);
    }
  }
  else
  {
    if (d3 < 0)
    {
      // Case B
      Vec3 ac = v1 + (d1/(d1 - d3))*(v3 - v1);
      output += tetrahedronVolume(ab, v2, v3, p);
      output += tetrahedronVolume(ab, v3, ac, p);
    }
    else
    {
      // Case A
      Vec3 bc = v2 + (d2/(d2 - d3))*(v3 - v2);
      output += tetrahedronVolume(ab, v2, bc, p);
    }
  }
  return output;
}

//////////////////////////////////////////////////////
Polyhedron::Volume Polyhedron::submergedVolume(const Vec3 &x,
    const ignition::math::Quaterniond &q, Plane &plane)
{
  // transform the plane into the polyhedron frame
  auto qt = q.Inverse();
  auto normal = qt.RotateVector(plane.normal);
  double offset = plane.offset - plane.normal.Dot(x);

  // compute vertex heights relative to surface
  std::vector<double> ds(vertices.size());
  int numSubmerged = 0;
  int sampleVert = 0;
  for (size_t i=0; i<vertices.size(); ++i)
  {
    ds[i] = normal.Dot(vertices[i]) - offset;
    if (ds[i] < -EPSILON)
    {
      ++numSubmerged;
      sampleVert = i;
    }
  }

  // if no submerged vertices return
  if (numSubmerged == 0)
  {
    return Volume{};
  }

  // Find a point on the water surface. Project a submerged point to
  // get improved accuracy. This point serves as the point of origin for
  // computing all the tetrahedron volumes. Since this point is on the
  // surface, all of the surface faces get zero volume tetrahedrons.
  // This way the surface polygon does not need to be considered.
  Vec3 p = vertices[sampleVert] - ds[sampleVert] * normal;

  // compute the contribution of each triangle
  Volume output;
  for (const auto& face : faces)
  {
    Vec3 v1 = vertices[face.i1];
    Vec3 v2 = vertices[face.i2];
    Vec3 v3 = vertices[face.i3];
    double d1 = ds[face.i1];
    double d2 = ds[face.i2];
    double d3 = ds[face.i3];

    if (d1 * d2 < 0)
    { // v1-v2 crosses the plane
      output += clipTriangle(v1, v2, v3, d1, d2, d3, p);
    }
    else if (d1 * d3 < 0)
    { // v1-v3 crosses the plane
      output += clipTriangle(v3, v1, v2, d3, d1, d2, p);
    }
    else if (d2 * d3 < 0)
    { // v2-v3 crosses the plane
      output += clipTriangle(v2, v3, v1, d2, d3, d1, p);
    }
    else if (d1 < 0 || d2 < 0 || d3 < 0)
    { // fully submerged
      output += tetrahedronVolume(v1, v2, v3, p);
    }
  }

  // small submerged slivers may have rounding error leading to
  // a zero or negative volume. If so, then return a result of zero.
  if (output.volume <= EPSILON)
  {
    return Volume{};
  }

  // normalize the centroid by the total volume
  output.centroid *= 1.0 / output.volume;
  // transform the centroid into world coordinates
  output.centroid = x + q.RotateVector(output.centroid);
  // if centroid is very small make it zero
  output.centroid[0] = ::fabs(output.centroid[0]) < EPSILON ? 0 : output.centroid[0];
  output.centroid[1] = ::fabs(output.centroid[1]) < EPSILON ? 0 : output.centroid[1];
  output.centroid[2] = ::fabs(output.centroid[2]) < EPSILON ? 0 : output.centroid[2];

  return output;
}