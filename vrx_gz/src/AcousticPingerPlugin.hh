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

#ifndef VRX_ACOUSTIC_PINGER_PLUGIN_HH_
#define VRX_ACOUSTIC_PINGER_PLUGIN_HH_

#include <memory>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

namespace vrx
{
  /// \brief Implements a range and bearing pinger system.  This assumes that
  /// the ppinger localisation has a mechanism for estimating the range and
  /// bearing of the pinger. Pinger estimates are published using messages.
  ///
  /// Bearings are reported in the vehicle frame and coordinate system:
  /// the x-axis is towards the vehicle’s nose, the y-axis is towards the port
  /// side, and the z-axis points upwards. Following the right-hand rule,
  /// bearing angles are measured counter-clockwise beginning from the x-axis.
  /// See https://github.com/osrf/vrx/wiki/frame_conventions#acoustic-pinger .
  ///
  /// Accepts the following SDF parameters:
  /// <frame_id> Tf frame of the sensor message. Used as part of the sensor
  ///            message publication.
  /// <topic> Name of the topic that the sensor message will be published on.
  ///         Defaults to "/pinger/range_bearing".
  /// <set_position_topic> Name of the topic that is used to set the position
  ///                      of the simulated pinger sensor.
  ///                      Defaults to "/pinger/set_pinger_position".
  /// <position> Position of the simulated pinger. Defaults to origin.
  /// <update_rate> Rate of simulated sensor messages. Defaults to 1Hz.
  /// <range_noise> Noise model for the range to the simulated pinger.
  /// <bearing_noise> Noise model for the bearing to the simulated pinger.
  /// <elevation_noise> Noise model for the elevation to the simulated pinger.
  class AcousticPingerPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor.
    public: AcousticPingerPlugin();

    /// \brief Destructor.
    public: ~AcousticPingerPlugin() override = default;

    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
#endif
