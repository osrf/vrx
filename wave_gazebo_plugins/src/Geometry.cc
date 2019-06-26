/*
 * Copyright (C) 2019  Rhys Mainwaring
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

#include <cmath>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include "wave_gazebo_plugins/Geometry.hh"

namespace asv
{
  /////////////////////////////////////////////////
  ignition::math::Vector2d
  Geometry::Normalize(const ignition::math::Vector2d& _v)
  {
    if (_v == ignition::math::Vector2d::Zero)
      return _v;
    else
      return _v/_v.Length();
  }

  /////////////////////////////////////////////////
  ignition::math::Vector3d
  Geometry::Normalize(const ignition::math::Vector3d& _v)
  {
    if (_v == ignition::math::Vector3d::Zero)
      return _v;
    else
      return _v/_v.Length();
  }

  /////////////////////////////////////////////////
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
}
