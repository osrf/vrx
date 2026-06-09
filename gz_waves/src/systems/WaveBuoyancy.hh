/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef GZ_SIM_SYSTEMS_WAVEBUOYANCY_HH_
#define GZ_SIM_SYSTEMS_WAVEBUOYANCY_HH_

#include <memory>

#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>

namespace gz::sim::systems
{
  /// \brief A buoyancy plugin that applies vertical buoyant forces at a set
  /// of sample points along a horizontal cylindrical hull, reading the
  /// world's `Wavefield` component for the wave-displaced surface.
  ///
  /// Successor to old VRX `vrx::Surface`. Same modeling assumptions
  /// (horizontal cylinder, vertical force, no roll/pitch buoyancy tilt) with
  /// two bugs fixed:
  ///   1. The submerged-depth clamp is `[0, 2·hull_radius]` (full diameter),
  ///      not `[0, hull_radius]` (half), so a fully submerged hull produces
  ///      true Archimedean buoyancy `ρ·g·V` instead of half of it.
  ///   2. The wavefield is read from the world ECM component; there is no
  ///      per-plugin Wavefield copy, no topic subscription, no mutex.
  ///
  /// ## SDF parameters
  ///
  ///   - `<link_name>` (string, required): link the buoyant forces act on.
  ///   - `<hull_length>` (double [m], 4.9): cylinder length the points span.
  ///   - `<hull_radius>` (double [m], 0.213): cylinder radius — sets each
  ///     point's submerged area and the [0, 2*radius] depth clamp.
  ///   - `<fluid_level>` (double [m], 0.0): still-water surface Z.
  ///   - `<fluid_density>` (double [kg/m^3], 1000.0): water density.
  ///   - `<points>`/`<point>` (vec3 [m], required): hull sample points in the
  ///     link frame; the hull length is divided evenly among them.
  ///
  /// \verbatim
  /// <plugin filename="gz-sim-wave-buoyancy-system"
  ///         name="gz::sim::systems::WaveBuoyancy">
  ///   <link_name>base_link</link_name>
  ///   <hull_length>4.9</hull_length>
  ///   <hull_radius>0.213</hull_radius>
  ///   <fluid_level>0</fluid_level>
  ///   <fluid_density>1000</fluid_density>
  ///   <points>
  ///     <point>1.225  1.2 0</point>
  ///     <point>1.225 -1.2 0</point>
  ///     <point>-1.225  1.2 0</point>
  ///     <point>-1.225 -1.2 0</point>
  ///   </points>
  /// </plugin>
  /// \endverbatim
  class WaveBuoyancy : public System,
                      public ISystemConfigure,
                      public ISystemPreUpdate
  {
    public: WaveBuoyancy();
    public: ~WaveBuoyancy() override;

    public: void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &_eventMgr) override;

    public: void PreUpdate(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}  // namespace gz::sim::systems

#endif  // GZ_SIM_SYSTEMS_WAVEBUOYANCY_HH_
