#pragma once

#include <cassert>
#include <utility>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

namespace buoyancy
{

  using Vec3 = ignition::math::Vector3d;

  struct Face
  {
    Face() {}
    Face(int i1, int i2, int i3) : i1(i1), i2(i2), i3(i3) {}
    int i1, i2, i3;
  };

  struct Plane
  {
    Vec3 normal;
    float offset;
  };

  struct Polyhedron
  {
    std::vector<Vec3> verts;
    std::vector<Face> faces;
  };


  buoyancy::Polyhedron make_cube(double x, double y, double z)
  {
    buoyancy::Polyhedron cube;

    for(int i=0; i<2; i++) {
      for(int j=0; j<2; j++) {
        for(int k=0; k<2; k++) {
          cube.verts.emplace_back(ignition::math::Vector3d(i*x - x/2.0, j*y - y/2.0, k*z - z/2.0));
        }
      }
    }

    cube.faces.emplace_back(buoyancy::Face(0,6,4));
    cube.faces.emplace_back(buoyancy::Face(0,2,6));
    cube.faces.emplace_back(buoyancy::Face(0,3,2));
    cube.faces.emplace_back(buoyancy::Face(0,1,3));
    cube.faces.emplace_back(buoyancy::Face(2,7,6));
    cube.faces.emplace_back(buoyancy::Face(2,3,7));
    cube.faces.emplace_back(buoyancy::Face(4,6,7));
    cube.faces.emplace_back(buoyancy::Face(4,7,5));
    cube.faces.emplace_back(buoyancy::Face(0,4,5));
    cube.faces.emplace_back(buoyancy::Face(0,5,1));
    cube.faces.emplace_back(buoyancy::Face(1,5,7));
    cube.faces.emplace_back(buoyancy::Face(1,7,3));

    return cube;
  }

  Polyhedron make_cylinder(double r, double l, int n)
  {
    assert(n > 4);

    buoyancy::Polyhedron cylinder;

    // generate all vertices
    double angle_step = 2.0 * M_PI / n;
    double l_2 = l / 2.0;
    cylinder.verts.resize(2*n+2);
    cylinder.verts[0] = Vec3{0,0,-l_2};
    for(int i=1; i<=n; i++) {
      double x = r * ::sin(angle_step * (i-1));
      double y = r * ::cos(angle_step * (i-1));
      // bottom plate
      cylinder.verts[i] = Vec3{x, y, -l_2};
      // top plate
      cylinder.verts[i+n] = Vec3{x, y, l_2};
    }
    cylinder.verts[2*n+1] = Vec3{0,0,l_2};

    // generate all faces
    for(int i=1; i<=n; i++) { // bottom plate
      cylinder.faces.emplace_back(Face(0, i, (i%n)+1));
    }
    for(int i=1; i<=n; i++) { // walls
      cylinder.faces.emplace_back(Face(i+1, i, n+i));
      cylinder.faces.emplace_back(Face((i%n)+n, (i%n)+n+1, (i%n)+1));
    }
    for(int i=1; i<=n; i++) { // top plate
      cylinder.faces.emplace_back(Face(i+n, 2*n+1, (i%n)+n+1));
    }

    assert(cylinder.verts.size() == 2 * n + 2);
//    assert(cylinder.faces.size() == 4 * n);

    return cylinder;
  }


  /// \brief computes volume and centroid of tetrahedron (formed by triangle + arbitrary point)
  /// @param v1: point on triangle
  /// @param v2: point on triangle
  /// @param v3: point on triangle
  /// @param p: arbitrary point
  /// @return pair <volume, centroid>
  static std::pair<double, Vec3> tetrahedronVolume(const Vec3& v1, const Vec3& v2,
      const Vec3& v3, const Vec3& p = Vec3({0.0,0.0,0.0}))
  {
    Vec3 a = v2 - v1;
    Vec3 b = v3 - v1;
    Vec3 r = p - v1;

    double volume = (1/6.) * (b.Cross(a)).Dot(r);
    Vec3 centroid = 0.25 * volume * (v1 + v2 + v3 + p);
    return std::make_pair(volume, centroid);
  }

  std::pair<double, Vec3> computeVolume(Polyhedron& poly)
  {
    float volume = 0;
    Vec3 centroid = {0,0,0};
    Vec3 zero = {0,0,0};

    // Compute the contribution of each triangle.
    for (int i = 0; i < poly.faces.size(); ++i)
    {
      int i1 = poly.faces[i].i1;
      int i2 = poly.faces[i].i2;
      int i3 = poly.faces[i].i3;

      Vec3 v1 = poly.verts[i1];
      Vec3 v2 = poly.verts[i2];
      Vec3 v3 = poly.verts[i3];

      double v;
      Vec3 c;
      std::tie(v, c) = tetrahedronVolume(v1, v2, v3);
      volume += v;
      centroid += c;
    }
    return std::make_pair(volume, centroid);
  }

  // Clips a partially submerged triangle and returns the volume of the
  // resulting tetrahedrons and updates the centroid accumulator.
  static std::pair<double, Vec3> clipTriangle(const Vec3& v1, const Vec3& v2, const Vec3& v3,
      float d1, float d2, float d3, const Vec3& p = Vec3({0,0,0}))
  {
    assert(d1 * d2 < 0);
    // calculate the intersection point from a to b
    Vec3 ab = v1 + (d1/(d1 - d2))*(v2 - v1);

    double volume = 0;
    Vec3 centroid = Vec3{0,0,0};

    double v;
    Vec3 c;

    if (d1 < 0)
    {
      // b to c crosses the clipping plane
      if (d3 < 0)
      {
        // Case B - a quadrilateral or two triangles
        // Calculate intersection point from b to c.
        Vec3 bc = v2 + (d2/(d2 - d3))*(v3 - v2);
        std::tie(v, c) = tetrahedronVolume(ab, bc, v1, p);
        volume += v;
        centroid += c;
        std::tie(v, c) = tetrahedronVolume(bc, v3, v1, p);
        volume += v;
        centroid += c;
      }
      else
      {
        // Case A - a single triangle.
        Vec3 ac = v1 + (d1/(d1 - d3))*(v3 - v1);
        std::tie(v, c) = tetrahedronVolume(ab, ac, v1, p);
        volume += v;
        centroid += c;
      }
    }
    else
    {
      if (d3 < 0)
      {
        // Case B
        Vec3 ac = v1 + (d1/(d1 - d3))*(v3 - v1);
        std::tie(v, c) = tetrahedronVolume(ab, v2, v3, p);
        volume += v;
        centroid += c;
        std::tie(v, c) = tetrahedronVolume(ab, v3, ac, p);
        volume += v;
        centroid += c;
      }
      else
      {
        // Case A
        Vec3 bc = v2 + (d2/(d2 - d3))*(v3 - v2);
        std::tie(v, c) = tetrahedronVolume(ab, v2, bc, p);
        volume += v;
        centroid += c;
      }
    }

    return std::make_pair(volume, centroid);
  }

  // Computes the submerged volume and center of buoyancy of a polyhedron with
  // the water surface defined as a plane.
  static std::pair<double, Vec3> submergedVolume(const Vec3& x, const ignition::math::Quaterniond& q,
      Polyhedron& poly, Plane& plane)
  {
    Vec3 centroid{0,0,0};

    // Transform the plane into the polyhedron frame
    auto qt = q.Inverse();
    auto normal = qt.RotateVector(plane.normal);
    double offset = plane.offset - plane.normal.Dot(x);

    // Compute the vertex heights relative to the surface.
    double TINY_DEPTH = -1e-6;
    double* ds = new double[poly.verts.size()];

    // Compute the depth of each vertex.
    int numSubmerged = 0;
    int sampleVert = 0;
    for (int i = 0; i < poly.verts.size(); ++i)
    {
      ds[i] = normal.Dot(poly.verts[i]) - offset;
      if (ds[i] < TINY_DEPTH)
      {
        ++numSubmerged;
        sampleVert = i;
      }
    }

    // Return early if no vertices are submerged
    if (numSubmerged == 0)
    {
      delete [] ds;
      return std::make_pair(0, centroid);
    }

    // Find a point on the water surface. Project a submerged point to
    // get improved accuracy. This point serves as the point of origin for
    // computing all the tetrahedron volumes. Since this point is on the
    // surface, all of the surface faces get zero volume tetrahedrons. This
    // way the surface polygon does not need to be considered.
    Vec3 p = poly.verts[sampleVert] - ds[sampleVert]*normal;

    // Initialize volume and centroid accumulators.
    float volume = 0;

    double v;
    Vec3 c;

    // Compute the contribution of each triangle.
    for (int i = 0; i < poly.faces.size(); ++i)
    {
      int i1 = poly.faces[i].i1;
      int i2 = poly.faces[i].i2;
      int i3 = poly.faces[i].i3;

      Vec3 v1 = poly.verts[i1];
      float d1 = ds[i1];

      Vec3 v2 = poly.verts[i2];
      float d2 = ds[i2];

      Vec3 v3 = poly.verts[i3];
      float d3 = ds[i3];

      if (d1 * d2 < 0)
      {
        // v1-v2 crosses the plane
        std::tie(v, c) = clipTriangle(v1, v2, v3, d1, d2, d3, p);
        volume += v;
        centroid += c;
      }
      else if (d1 * d3 < 0)
      {
        // v1-v3 crosses the plane
        std::tie(v, c) = clipTriangle(v3, v1, v2, d3, d1, d2, p);
        volume += v;
        centroid += c;
      }
      else if (d2 * d3 < 0)
      {
        // v2-v3 crosses the plane
        std::tie(v, c) = clipTriangle(v2, v3, v1, d2, d3, d1, p);
        volume += v;
        centroid += c;
      }
      else if (d1 < 0 || d2 < 0 || d3 < 0)
      {
        // fully submerged
        std::tie(v, c) = tetrahedronVolume(v1, v2, v3, p);
        volume += v;
        centroid += c;
      }
    }

    // Small submerged slivers may have roundoff error leading to a zero or negative
    // volume. If so, then return a result of zero.
    double TINY_VOLUME = 1e-6;
    if (volume <= TINY_VOLUME)
    {
      delete [] ds;
      return std::make_pair(0, Vec3{0,0,0});
    }

    // Normalize the centroid by the total volume.
    c *= 1.0 / volume;

    // Transform the centroid into world coordinates.
    c = x + q.RotateVector(c);

    delete [] ds;
    return std::make_pair(volume, centroid);
  }

} // namespace buoyancy