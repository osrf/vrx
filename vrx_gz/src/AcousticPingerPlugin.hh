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
  /// \brief Implements a simulated range and bearing pinger localisation system
  ///
  /// Implements a range and bearing pinger system.  This assumes that the pinger
  /// localisation has a mechanism for estimating the range and bearing
  /// of the pinger.  Pinger estimates are published using a custom message to the
  /// ROS system along with a standard header.  This should allow the tf library
  /// to transform the sensor reading between frames.
  ///
  /// Accepts the following SDF parameters:
  /// <robotNamespace> - Set the namespace of the robot. Used to setup the ROS
  ///   nodehandle.
  /// <frameId> - Tf frame of the sensor message. Used as part of the sensor
  ///   message publication.
  /// <topicName> - Name of the topic that the sensor message will be published on
  /// <setPositionTopicName> - Name of the topic that is used to set the position
  ///   of the simulated pinger sensor.
  /// <position> - Position of the simulated pinger. Defaults to origin.
  /// <updateRate> - Rate of simulated sensor messages.
  /// <rangeNoise> - Noise model for the range to the simulated pinger.
  /// <bearingNoise> - Noise model for the bearing to the simulated pinger.
  /// <elevationNoise> - Noise model for the elevation to the simulated pinger.
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
