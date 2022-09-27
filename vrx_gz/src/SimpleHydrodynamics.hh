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
#ifndef VRX_SIMPLE_HYDRODYNAMICS_HH_
#define VRX_SIMPLE_HYDRODYNAMICS_HH_

#include <memory>
#include <gz/sim/System.hh>
#include <sdf/sdf.hh>

namespace vrx
{
  // Forward declaration
  class SimpleHydrodynamicsPrivate;

  /// \brief This class provides hydrodynamic behaviour for underwater vehicles
  /// It is shamelessly based off Brian Bingham's
  /// [plugin for VRX](https://github.com/osrf/vrx).
  /// which in turn is based of Fossen's equations described in "Guidance and
  /// Control of Ocean Vehicles" [1]. The class should be used together with a
  /// buoyancy plugin to help simulate behaviour of maritime vehicles.
  /// Hydrodynamics refers to the behaviour of bodies in water. It includes
  /// forces like linear and quadratic drag, buoyancy (not provided by this
  /// plugin), etc.
  ///
  /// ## Required system parameters
  ///
  ///  * <link_name> - The link of the model that is being subject to
  ///     hydrodynamic forces.
  ///
  /// ## Optional system parameters
  /// The exact description of these parameters can be found on p. 37 of
  /// Fossen's book. They are used to calculate added mass, linear and quadratic
  /// drag and coriolis force.
  ///
  ///  * <xDotU> - Added mass in x direction [kg]
  ///  * <yDotV> - Added mass in y direction [kg]
  ///  * <zDotW> - Added mass in z direction [kg]
  ///  * <kDotP> - Added mass in roll direction [kgm^2]
  ///  * <mDotQ> - Added mass in pitch direction [kgm^2]
  ///  * <nDotR> - Added mass in yaw direction [kgm^2]
  ///  * <xUU>   - Stability derivative, 2nd order, x component [kg/m]
  ///  * <xU>    - Stability derivative, 1st order, x component [kg]
  ///  * <yVV>   - Stability derivative, 2nd order, y component [kg/m]
  ///  * <yV>    - Stability derivative, 1st order, y component [kg]
  ///  * <zWW>   - Stability derivative, 2nd order, z component [kg/m]
  ///  * <zW>    - Stability derivative, 1st order, z component [kg]
  ///  * <kPP>   - Stability derivative, 2nd order, roll component [kg/m^2]
  ///  * <kP>    - Stability derivative, 1st order, roll component [kg/m]
  ///  * <mQQ>   - Stability derivative, 2nd order, pitch component [kg/m^2]
  ///  * <mQ>    - Stability derivative, 1st order, pitch component [kg/m]
  ///  * <nRR>   - Stability derivative, 2nd order, yaw component [kg/m^2]
  ///  * <nR>    - Stability derivative, 1st order, yaw component [kg/m]
  ///
  /// # Example
  /// <plugin
  ///   filename="libSimpleHydrodynamics.so"
  ///   name="gz::sim::systems::SimpleHydrodynamics">
  ///   <link_name>base_link</link_name>
  ///   <!-- Added mass -->
  ///   <xDotU>0.0</xDotU>
  ///   <yDotV>0.0</yDotV>
  ///   <nDotR>0.0</nDotR>
  ///   <!-- Linear and quadratic drag -->
  ///   <xU>51.3</xU>
  ///   <xUU>72.4</xUU>
  ///   <yV>40.0</yV>
  ///   <yVV>0.0</yVV>
  ///   <zW>500.0</zW>
  ///   <kP>50.0</kP>
  ///   <mQ>50.0</mQ>
  ///   <nR>400.0</nR>
  ///   <nRR>0.0</nRR>
  /// </plugin>
  ///
  /// # Citations
  /// [1] Fossen, Thor I. _Guidance and Control of Ocean Vehicles_.
  ///    United Kingdom: Wiley, 1994.
  class SimpleHydrodynamics
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor.
    public: SimpleHydrodynamics();

    /// \brief Destructor.
    public: ~SimpleHydrodynamics() override = default;

    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    private: std::unique_ptr<SimpleHydrodynamicsPrivate> dataPtr;
  };
}

#endif
