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

#ifndef VRX_POLYHEDRONVOLUME_HH_
#define VRX_POLYHEDRONVOLUME_HH_

#include <vector>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>

namespace vrx
{
  /// \brief Represents a plane as a normal and offset.
  struct Plane
  {
    /// \brief Initializes plane at z=0.
    Plane();

    /// \brief Vector3 normal to plane.
    gz::math::Vector3d normal;

    /// \brief Offset w.r.t. normal.
    float offset;
  };

  /// \brief Represents output volume with centroid.
  struct Volume
  {
    Volume();

    /// \brief Overloads += for volume object.
    Volume &operator+=(const Volume &_rhs);

    /// \brief Submerged volume of shape.
    double volume;

    /// \brief Vector3 representing volume centroid.
    gz::math::Vector3d centroid;
  };

  /// \brief Submerged volume calculation using polyhedron
  /// based on: Exact Buoyancy for Polyhedra by Eric Catto.
  /// \ref Erin Catto. “Exact Buoyancy for Polyhedra”.
  ///      In Game Programming Gems 6, Charles River Media, 2006, pp. 175–187.
  class Polyhedron
  {
    /// \brief Store vertex index for a triangular face.
    public: struct Face
    {
      /// \brief Default constructor.
      Face() = default;

      /// \brief Constructor.
      /// \param[in] _i1 Index 1.
      /// \param[in] _i2 Index 2.
      /// \param[in] _i3 Index 3.
      Face(int _i1,
           int _i2,
           int _i3);

      /// \brief Index of vertex.
      public: int i1;

      /// \brief Index of vertex.
      public: int i2;

      /// \brief Index of vertex.
      public: int i3;
    };

    /// \brief Generate a cube polyhedron centered at origin.
    /// \param[in] _x Length of cube.
    /// \param[in] _y Width of cube.
    /// \param[in] _z Height of cube.
    /// \return Polyhedron object.
    public: static Polyhedron makeCube(double _x,
                                       double _y,
                                       double _z);

    /// \brief Generate a cylinder polyhedron centered at origin.
    /// \param[in] _r Radius of cylinder.
    /// \param[in] _l Length of cylinder.
    /// \param[in] _n Number of segments.
    /// \return Polyhedron object.
    public: static Polyhedron makeCylinder(double _r,
                                           double _l,
                                           int    _n);

    /// \brief Compute full volume and center of buoyancy of the polyhedron.
    /// \return Volume object with volume and centroid.
    public: Volume ComputeFullVolume() const;

    /// \brief Compute submerged volume and center of buoyancy of a polyhedron.
    /// \param[in] _x Our position.
    /// \param[in] _q Our orientation (quaternions).
    /// \param[in] _plane Water surface defined as a plane.
    /// \return Volume object with volume and centroid (relative to world).
    public: Volume SubmergedVolume(const gz::math::Vector3d &_x,
                                   const gz::math::Quaterniond &_q,
                                   const Plane &_plane) const;

    /// \brief Computes volume and centroid of tetrahedron.
    /// tetrahedron formed by triangle + arbitrary point.
    /// \param[in] _v1 Point on triangle.
    /// \param[in] _v2 Point on triangle.
    /// \param[in] _v3 Point on triangle.
    /// \param[in] _p Arbitrary point.
    /// \return Volume object with volume and centroid.
    private: static Volume tetrahedronVolume(
                                      const gz::math::Vector3d &_v1,
                                      const gz::math::Vector3d &_v2,
                                      const gz::math::Vector3d &_v3,
                                      const gz::math::Vector3d &_p =
                                        gz::math::Vector3d::Zero);

    /// \brief Clips a partially submerged triangle.
    /// \param[in] _v1 Point on triangle.
    /// \param[in] _v2 Point on triangle.
    /// \param[in] _v3 Point on triangle.
    /// \param[in] _d1 Distance of point v1 to the splitting plane.
    /// \param[in] _d2 Distance of point v2 to the splitting plane.
    /// \param[in] _d3 Distance of point v3 to the splitting plane.
    /// \return Volume object for clipped tetrahedron.
    private: static Volume clipTriangle(const gz::math::Vector3d &_v1,
                                        const gz::math::Vector3d &_v2,
                                        const gz::math::Vector3d &_v3,
                                        double _d1,
                                        double _d2,
                                        double _d3,
                                        const gz::math::Vector3d &_p =
                                          gz::math::Vector3d::Zero);

    /// \brief Object vertices.
    private: std::vector<gz::math::Vector3d> vertices;

    /// \brief Object faces.
    private: std::vector<Face> faces;

    /// \brief Values below this are zeroed out.
    private: const double EPSILON = 1e-6;
  };  // class Polyhedron
}

#endif
