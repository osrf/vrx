// Copyright (C) 2019  Rhys Mainwaring
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "wave_gazebo_plugins/Geometry.hh"

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <cmath>

namespace asv 
{
  ignition::math::Vector2d Geometry::Normalize(const ignition::math::Vector2d& _v)
  {
    if (_v == ignition::math::Vector2d::Zero)
      return _v;
    else
      return _v/_v.Length(); 
  }

  ignition::math::Vector3d Geometry::Normalize(const ignition::math::Vector3d& _v)
  {
    if (_v == ignition::math::Vector3d::Zero)
      return _v;
    else
      return _v/_v.Length(); 
  }

  ignition::math::Vector3d Geometry::Normal(
    const ignition::math::Vector3d& _p0,
    const ignition::math::Vector3d& _p1,
    const ignition::math::Vector3d& _p2
  )
  {
    auto n = ignition::math::Vector3d::Normal(_p0, _p1, _p2);
    if (n == ignition::math::Vector3d::Zero)
      return n;
    else
      return n/n.Length(); 
  }
  
} // namespace asv

