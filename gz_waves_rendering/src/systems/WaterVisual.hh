/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef GZ_SIM_SYSTEMS_WATERVISUAL_HH_
#define GZ_SIM_SYSTEMS_WATERVISUAL_HH_

#include <memory>

#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>

namespace gz::sim::systems
{
  /// \brief A rendering system that drives a water-surface visual with the
  /// FFT water vertex/fragment shaders, fed by the world's `Wavefield`
  /// component.
  ///
  /// Attach to a `<visual>` element. The plugin finds its visual, attaches a
  /// material with the water vertex shader, and uploads parameters from
  /// the wavefield component whenever its generation counter changes. The
  /// sim-time uniform is pushed every frame.
  ///
  /// ## SDF parameters
  /// \verbatim
  /// <plugin filename="gz-sim-water-visual-system"
  ///         name="gz::sim::systems::WaterVisual">
  ///   <shader>
  ///     <fft_vertex>shaders/fft_water_vs_330.glsl</fft_vertex>
  ///     <fragment>shaders/water_fs_330.glsl</fragment>
  ///     <parameters>
  ///       <rescale>0.125</rescale>
  ///       <bumpScale>75 75</bumpScale>
  ///       <bumpSpeed>0.01 0.0</bumpSpeed>
  ///       <hdrMultiplier>0.4</hdrMultiplier>
  ///       <fresnelPower>5.0</fresnelPower>
  ///       <shallowColor>0 0.1 0.2 1.0</shallowColor>
  ///       <deepColor>0 0.05 0.2 1.0</deepColor>
  ///     </parameters>
  ///   </shader>
  ///   <textures>
  ///     <bumpMap>textures/wave_normals.dds</bumpMap>
  ///     <cubeMap>textures/skybox_lowres.dds</cubeMap>
  ///   </textures>
  /// </plugin>
  /// \endverbatim
  class WaterVisual : public System,
                      public ISystemConfigure,
                      public ISystemPreUpdate
  {
    public: WaterVisual();
    public: ~WaterVisual() override;

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

#endif  // GZ_SIM_SYSTEMS_WATERVISUAL_HH_
