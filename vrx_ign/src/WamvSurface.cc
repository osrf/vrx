/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>

#include "WamvSurface.hh"

using namespace ignition;
using namespace vrx;

/// \brief Private Surface data class.
class vrx::WamvSurface::Implementation
{
};


//////////////////////////////////////////////////
WamvSurface::WamvSurface()
  : dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
double WamvSurface::BuoyancyAtPoint(const gazebo::UpdateInfo &/*_info*/,
    const math::Vector3d &/*_point*/, double _deltaZ,
    gazebo::EntityComponentManager &/*_ecm*/)
{
  const uint8_t kNumPontoons = 2u;

  // Buoyancy force at this point.
  return this->CircleSegment(this->HullRadius(), _deltaZ) *
    this->VehicleLength() / kNumPontoons *
    -this->Gravity().Z() * this->FluidDensity();
}

IGNITION_ADD_PLUGIN(WamvSurface,
                    gazebo::System,
                    Surface::ISystemConfigure,
                    Surface::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(vrx::WamvSurface,
                          "vrx::WamvSurface")
