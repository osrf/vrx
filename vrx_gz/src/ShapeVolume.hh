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

#ifndef VRX_SHAPEVOLUME_HH_
#define VRX_SHAPEVOLUME_HH_

#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <gz/math/Pose3.hh>
#include <sdf/sdf.hh>

#include "PolyhedronVolume.hh"

namespace vrx
{
  /// \brief Type of geometry shape.
  enum class ShapeType
  {
    None,
    Box,
    Sphere,
    Cylinder
  };

  /// \brief Parent shape object for volume objects.
  struct ShapeVolume
  {
    /// \brief Default destructor.
    virtual ~ShapeVolume() = default;

    /// \brief Factory method for shape. Parses a shape object from sdf data
    /// \param[in] _sdf geometry SDF element
    /// \return A pointer to a new ShapeVolume.
    static std::unique_ptr<ShapeVolume> makeShape(const sdf::ElementPtr _sdf);

    /// \brief Display string for shape object.
    /// \return A string representation of the ShapeVolume.
    virtual std::string Display() const;

    /// \brief Calculates volume + centroid of submerged shape.
    /// If the shape is out of water returns Volume{}.
    /// \param[in] _pose World pose of volume.
    /// \param[in] _fluidLevel Height of fluid. Default is 0.
    /// \return Volume object with volume + centroid (relative to world).
    virtual Volume CalculateVolume(const gz::math::Pose3d &_pose,
                                   double _fluidLevel) const = 0;

    /// \brief Type of shape.
    ShapeType type;

    /// \brief Full volume of object.
    double volume;

    /// \brief Average length of object.
    /// Estimate used for drag torque calculation.
    double averageLength;
  };

  typedef std::unique_ptr<ShapeVolume> ShapeVolumePtr;

  /// \brief Box shape volume.
  struct BoxVolume : public ShapeVolume
  {
    /// \brief Default constructor.
    /// \param[in] _x Length.
    /// \param[in] _y Width.
    /// \param[in] _z Height.
    explicit BoxVolume(double _x,
                       double _y,
                       double _z);

    // Documentation inherited.
    std::string Display() const override;

    // Documentation inherited.
    Volume CalculateVolume(const gz::math::Pose3d &_pose,
                           double _fluidLevel) const override;

    /// \brief Length.
    double x;

    /// \brief Width.
    double y;

    /// \brief Height.
    double z;

    /// \brief Polyhedron defining a box.
    private: Polyhedron polyhedron;
  };

  /// \brief Cylinder shape volume.
  struct CylinderVolume : public ShapeVolume
  {
    /// \brief Default constructor
    /// \param[in] _r Radius.
    /// \param[in] _l Length.
    explicit CylinderVolume(double _r,
                            double _l);

    // Documentation inherited.
    std::string Display() const override;

    // Documentation inherited.
    Volume CalculateVolume(const gz::math::Pose3d &_pose,
                           double _fluidLevel) const override;

    /// \brief Radius of cylinder.
    double r;

    /// \brief Height of cylinder.
    double h;

    /// \brief Polyhedron defining a cylinder.
    private: Polyhedron polyhedron;
  };

  /// \brief Sphere shape volume.
  struct SphereVolume : public ShapeVolume
  {
    /// \brief Default constructor.
    /// \param[in] _r Radius.
    explicit SphereVolume(double _r);

    // Documentation inherited.
    std::string Display() const override;

    // Documentation inherited.
    Volume CalculateVolume(const gz::math::Pose3d &_pose,
                           double _fluidLevel) const override;

    /// \brief Radius of sphere.
    double r;
  };

  /// \brief Custom exception for parsing errors.
  struct ParseException : public std::exception
  {
    ParseException(const char* shape, const char* message)
      : output_("")
    {
      std::stringstream ss;
      ss << "Parse error for <" << shape << ">: " << message;
      // cppcheck-suppress useInitializationList
      this->output_ = ss.str();
    }

    const char* what() const throw()
    {
      return this->output_.c_str();
    }

    private: std::string output_;
  };
}

#endif
