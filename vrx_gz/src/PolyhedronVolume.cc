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

#include <cassert>
#include "PolyhedronVolume.hh"

using namespace vrx;
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
Volume::Volume()
  : volume(0.0), centroid(gz::math::Vector3d({0, 0, 0}))
{
}

//////////////////////////////////////////////////////
Volume &Volume::operator+=(const Volume &_rhs)
{
  this->volume += _rhs.volume;
  this->centroid += _rhs.centroid;
  return *this;
}

//////////////////////////////////////////////////////
Polyhedron Polyhedron::makeCube(double _x, double _y, double _z)
{
  Polyhedron cube;
  // generate vertices
  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 2; ++j)
    {
      for (int k = 0; k < 2; ++k)
      {
        cube.vertices.emplace_back(
            gz::math::Vector3d(i * _x - _x / 2.0,
                j * _y - _y / 2.0, k * _z - _z / 2.0));
      }
    }
  }
  // generate faces
  cube.faces.emplace_back(Face(0, 6, 4));
  cube.faces.emplace_back(Face(0, 2, 6));
  cube.faces.emplace_back(Face(0, 3, 2));
  cube.faces.emplace_back(Face(0, 1, 3));
  cube.faces.emplace_back(Face(2, 7, 6));
  cube.faces.emplace_back(Face(2, 3, 7));
  cube.faces.emplace_back(Face(4, 6, 7));
  cube.faces.emplace_back(Face(4, 7, 5));
  cube.faces.emplace_back(Face(0, 4, 5));
  cube.faces.emplace_back(Face(0, 5, 1));
  cube.faces.emplace_back(Face(1, 5, 7));
  cube.faces.emplace_back(Face(1, 7, 3));

  return cube;
}

//////////////////////////////////////////////////////
Polyhedron Polyhedron::makeCylinder(double _r, double _l, int _n)
{
  assert(_n > 4);

  Polyhedron cylinder;

  // Generate all vertices.
  double angle_step = 2.0 * M_PI / _n;
  double l_2 = _l / 2.0;
  cylinder.vertices.resize(2 * _n + 2);
  cylinder.vertices[0] = gz::math::Vector3d{0, 0, -l_2};
  for (int i = 1; i <= _n; ++i)
  {
    double x = _r * ::sin(angle_step * (i - 1));
    double y = _r * ::cos(angle_step * (i - 1));
    // bottom plate
    cylinder.vertices[i] = gz::math::Vector3d{x, y, -l_2};
    // top plate
    cylinder.vertices[i + _n] = gz::math::Vector3d{x, y, l_2};
  }
  cylinder.vertices[2 * _n + 1] = gz::math::Vector3d{0, 0, l_2};

  // generate all faces.
  for (int i = 1; i <= _n; ++i)
  {
    // bottom plate.
    cylinder.faces.emplace_back(Face(0, i, (i % _n) + 1));
  }
  for (int i = 1; i <= _n; ++i)
  {
    // walls
    cylinder.faces.emplace_back(Face(i + 1, i, _n + i));
    cylinder.faces.emplace_back(Face(
      (i % _n) + _n, (i % _n) + _n + 1, (i % _n) + 1));
  }
  for (int i = 1; i <= _n; ++i)
  {
    // top plate
    cylinder.faces.emplace_back(Face(i + _n, 2 * _n + 1, (i % _n) + _n + 1));
  }

  assert(cylinder.vertices.size() == 2 * _n + 2);
  assert(cylinder.faces.size() == 4 * _n);

  return cylinder;
}

//////////////////////////////////////////////////////
Volume Polyhedron::ComputeFullVolume() const
{
  Volume output;
  // Compute the contribution of each triangle face
  for (const auto& face : faces)
  {
    gz::math::Vector3d v1 = vertices[face.i1];
    gz::math::Vector3d v2 = vertices[face.i2];
    gz::math::Vector3d v3 = vertices[face.i3];
    output += tetrahedronVolume(v1, v2, v3);
  }
  return output;
}

//////////////////////////////////////////////////////
Volume Polyhedron::SubmergedVolume(const gz::math::Vector3d &_x,
    const gz::math::Quaterniond &_q, const Plane &_plane) const
{
  // Transform the plane into the polyhedron frame.
  auto qt = _q.Inverse();
  auto normal = qt.RotateVector(_plane.normal);
  double offset = _plane.offset - _plane.normal.Dot(_x);

  // Compute vertex heights relative to surface.
  std::vector<double> ds(vertices.size());
  int numSubmerged = 0;
  int sampleVert = 0;
  for (size_t i = 0; i < vertices.size(); ++i)
  {
    ds[i] = normal.Dot(vertices[i]) - offset;
    if (ds[i] < -EPSILON)
    {
      ++numSubmerged;
      sampleVert = i;
    }
  }

  // If no submerged vertices return.
  if (numSubmerged == 0)
  {
    return Volume{};
  }

  // Find a point on the water surface. Project a submerged point to
  // get improved accuracy. This point serves as the point of origin for
  // computing all the tetrahedron volumes. Since this point is on the
  // surface, all of the surface faces get zero volume tetrahedrons.
  // This way the surface polygon does not need to be considered.
  gz::math::Vector3d p = vertices[sampleVert] - ds[sampleVert] * normal;

  // Compute the contribution of each triangle.
  Volume output;
  for (const auto &face : faces)
  {
    gz::math::Vector3d v1 = vertices[face.i1];
    gz::math::Vector3d v2 = vertices[face.i2];
    gz::math::Vector3d v3 = vertices[face.i3];
    double d1 = ds[face.i1];
    double d2 = ds[face.i2];
    double d3 = ds[face.i3];

    if (d1 * d2 < 0)
    {
      // v1-v2 crosses the plane
      output += clipTriangle(v1, v2, v3, d1, d2, d3, p);
    }
    else if (d1 * d3 < 0)
    {
      // v1-v3 crosses the plane
      output += clipTriangle(v3, v1, v2, d3, d1, d2, p);
    }
    else if (d2 * d3 < 0)
    {
      // v2-v3 crosses the plane
      output += clipTriangle(v2, v3, v1, d2, d3, d1, p);
    }
    else if (d1 < 0 || d2 < 0 || d3 < 0)
    {
      // fully submerged
      output += tetrahedronVolume(v1, v2, v3, p);
    }
  }

  // Small submerged slivers may have rounding error leading to
  // a zero or negative volume. If so, then return a result of zero.
  if (output.volume <= EPSILON)
  {
    return Volume{};
  }

  // Normalize the centroid by the total volume.
  output.centroid *= 1.0 / output.volume;
  // Transform the centroid into world coordinates.
  output.centroid = _x + _q.RotateVector(output.centroid);
  // If centroid is very small make it zero.
  output.centroid.X() = ::fabs(output.centroid[0]) < EPSILON ?
      0 : output.centroid.X();
  output.centroid.Y() = ::fabs(output.centroid[1]) < EPSILON ?
      0 : output.centroid.Y();
  output.centroid.Z() = ::fabs(output.centroid[2]) < EPSILON ?
      0 : output.centroid.Z();
  return output;
}

//////////////////////////////////////////////////////
Volume Polyhedron::tetrahedronVolume(const gz::math::Vector3d &_v1,
    const gz::math::Vector3d &_v2, const gz::math::Vector3d &_v3,
    const gz::math::Vector3d &_p)
{
  gz::math::Vector3d a = _v2 - _v1;
  gz::math::Vector3d b = _v3 - _v1;
  gz::math::Vector3d r = _p - _v1;

  Volume output;
  output.volume = (1 / 6.) * (b.Cross(a)).Dot(r);
  output.centroid = 0.25 * output.volume * (_v1 + _v2 + _v3 + _p);
  return output;
}

//////////////////////////////////////////////////////
Volume Polyhedron::clipTriangle(const gz::math::Vector3d &_v1,
    const gz::math::Vector3d &_v2, const gz::math::Vector3d &_v3,
    double _d1, double _d2, double _d3, const gz::math::Vector3d &_p)
{
  assert(_d1 * _d2 < 0);
  Volume output;

  // Calculate the intersection point from a to b.
  gz::math::Vector3d ab = _v1 + (_d1 / (_d1 - _d2)) * (_v2 - _v1);
  if (_d1 < 0)
  {
    // b to c crosses the clipping plane.
    if (_d3 < 0)
    {
      // Case B - a quadrilateral or two triangles
      // Calculate intersection point from b to c.
      gz::math::Vector3d bc = _v2 + (_d2 / (_d2 - _d3)) * (_v3 - _v2);
      output += tetrahedronVolume(ab, bc, _v1, _p);
      output += tetrahedronVolume(bc, _v3, _v1, _p);
    }
    else
    {
      // Case A - a single triangle.
      gz::math::Vector3d ac = _v1 + (_d1 / (_d1 - _d3)) * (_v3 - _v1);
      output += tetrahedronVolume(ab, ac, _v1, _p);
    }
  }
  else
  {
    if (_d3 < 0)
    {
      // Case B.
      gz::math::Vector3d ac = _v1 + (_d1 / (_d1 - _d3)) * (_v3 - _v1);
      output += tetrahedronVolume(ab, _v2, _v3, _p);
      output += tetrahedronVolume(ab, _v3, ac, _p);
    }
    else
    {
      // Case A.
      gz::math::Vector3d bc = _v2 + (_d2 / (_d2 - _d3)) * (_v3 - _v2);
      output += tetrahedronVolume(ab, _v2, bc, _p);
    }
  }
  return output;
}
