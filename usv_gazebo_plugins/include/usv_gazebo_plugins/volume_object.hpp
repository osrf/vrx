#pragma once

#include <cassert>
#include <utility>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ostream>

namespace buoyancy
{
  using Vec3 = ignition::math::Vector3d;

  /// \brief represents a plane as a normal and offset
  struct Plane
  {
    /// \brief initializes plane at z=0
    Plane();

    /// \brief vector3 normal to plane
    Vec3 normal;

    /// \brief offset w.r.t. normal
    float offset;
  };

  /// \brief Submerged volume calculation using polyhedron
  /// based on: Exact Buoyancy for Polyhedra by Eric Catto
  class Polyhedron
  {
  public:

    /// \brief Store vertex index for a triangular face
    struct Face
    {
      Face() = default;

      Face(int i1, int i2, int i3);

      /// \brief index of vertices
      int i1, i2, i3;
    };

    /// \brief Represents output volume with centroid
    struct Volume
    {
      Volume ();

      /// \brief overloads += for volume object
      Volume& operator+=(const Volume& rhs);

      /// \brief volume
      double volume;

      /// \brief vector3 representing volume centroid
      Vec3 centroid;
    };

    /// \brief generate a cube polyhedron centered at origin
    /// @param x: length of cube
    /// @param y: width of cube
    /// @param z: height of cube
    /// @return Polyhedron object
    static Polyhedron makeCube(double x, double y, double z);

    /// \brief generate a cylinder polyhedron centered at origin
    /// @param r: radius of cylinder
    /// @param l: length of cylinder
    /// @param n: number of segments
    /// @return Polyhedron object
    static Polyhedron makeCylinder(double r, double l, int n);

    /// \brief compute full volume and center of buoyancy of the polyhedron
    /// @return Volume object with volume and centroid
    Volume computeFullVolume();

    /// \brief compute submerge volume and center of buoyancy of a polyhedron
    /// @param x: our position
    /// @param q: our orientation (quaternions)
    /// @param plane: water surface defined as a plane
    /// @return Volume object with volume and centroid
    Volume submergedVolume(const Vec3& x, const ignition::math::Quaterniond& q,
        Plane& plane);

  private:
    /// \brief computes volume and centroid of tetrahedron
    /// tetrahedron formed by triangle + arbitrary point
    /// @param v1: point on triangle
    /// @param v2: point on triangle
    /// @param v3: point on triangle
    /// @param p: arbitrary point
    /// @return Volume object with volume and centroid
    static Volume tetrahedronVolume(const Vec3& v1, const Vec3& v2,
        const Vec3& v3, const Vec3& p = Vec3({0.,0.,0.}));

    /// \brief clips a partially submerged triangle
    /// @param v1: point on triangle
    /// @param v2: point on triangle
    /// @param v3: point on triangle
    /// @param d1: distance of point v1 to the splitting plane
    /// @param d2: distance of point v2 to the splitting plane
    /// @param d3: distance of point v3 to the splitting plane
    /// @return Volume object for clipped tetrahedron
    static Volume clipTriangle(const Vec3& v1, const Vec3& v2,
        const Vec3& v3, double d1, double d2, double d3,
        const Vec3& p = Vec3({0.,0.,0.}));

    /// \brief object vertices
    std::vector<Vec3> vertices;

    /// \brief object faces
    std::vector<Face> faces;

    /// \brief values below this are zeroed out
    const double EPSILON = 1e-6;

  }; // class Polyhedron

} // namespace buoyancy