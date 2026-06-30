/*
 * Copyright (C) 2026 Honu Robotics
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
  /// grid/displacement water vertex/fragment shaders, fed by the world's
  /// `Wavefield` component. The same grid path serves every engine (analytic
  /// Gerstner and FFT alike) via the shared `WaveField2D` heightmap.
  ///
  /// Attach to a `<visual>` element. The plugin finds its visual, attaches a
  /// material with the water vertex shader, and uploads parameters from
  /// the wavefield component whenever its generation counter changes. The
  /// sim-time uniform is pushed every frame.
  ///
  /// ## SDF parameters
  ///
  /// `<shader>` (required) names the GLSL files and tunes the surface look;
  /// `<textures>` supplies the bump/cube maps; the two tile tags control
  /// instanced tiling of the periodic wave field.
  ///
  ///   - `<shader>/<fft_vertex>` (path, required): vertex shader.
  ///   - `<shader>/<fragment>` (path, required): fragment shader.
  ///   - `<shader>/<parameters>/<rescale>` (float, 0.125): displacement/tangent scale.
  ///   - `<shader>/<parameters>/<bumpScale>` (vec2, 64 64): bumpmap tiling (x16 in the VS).
  ///   - `<shader>/<parameters>/<bumpSpeed>` (vec2, 0.01 0.01): bumpmap scroll [uv/s].
  ///   - `<shader>/<parameters>/<hdrMultiplier>` (float, 0.4): reflected-sky brightness.
  ///   - `<shader>/<parameters>/<fresnelPower>` (float, 5.0): Fresnel exponent.
  ///   - `<shader>/<parameters>/<roughness>` (float, 0.0): micro-surface roughness.
  ///   - `<shader>/<parameters>/<foamStrength>` (float, 0.7): whitecap blend amount (grid foam; off for the Gerstner engine).
  ///   - `<shader>/<parameters>/<foamThreshold>` (float, 0.25): half-width of the foam ramp in folding-metric space.
  ///   - `<shader>/<parameters>/<shallowColor>` (rgba, 0 0.1 0.3 1): shallow-water tint.
  ///   - `<shader>/<parameters>/<deepColor>` (rgba, 0 0.05 0.2 1): deep-water tint.
  ///   - `<textures>/<bumpMap>` (path): normal-perturbation texture.
  ///   - `<textures>/<cubeMap>` (path): reflection cubemap.
  ///   - `<tiles_radius>` (int, 2): render the tile at (2r+1)^2 offsets; 0 disables.
  ///   - `<tile_mesh_size>` (double [m], 200.0): per-tile mesh extent.
  ///
  /// \verbatim
  /// <plugin filename="gz-sim-water-visual-system"
  ///         name="gz::sim::systems::WaterVisual">
  ///   <shader>
  ///     <fft_vertex>shaders/fft_water_vs_330.glsl</fft_vertex>
  ///     <fragment>shaders/water_fs_330.glsl</fragment>
  ///     <parameters>
  ///       <rescale>0.5</rescale>
  ///       <bumpScale>64 64</bumpScale>
  ///       <bumpSpeed>0.01 0.01</bumpSpeed>
  ///       <hdrMultiplier>0.4</hdrMultiplier>
  ///       <fresnelPower>5.0</fresnelPower>
  ///       <shallowColor>0 0.1 0.3 1.0</shallowColor>
  ///       <deepColor>0 0.05 0.2 1.0</deepColor>
  ///     </parameters>
  ///   </shader>
  ///   <textures>
  ///     <bumpMap>textures/wave_normals.dds</bumpMap>
  ///     <cubeMap>textures/skybox_lowres.dds</cubeMap>
  ///   </textures>
  ///   <tiles_radius>2</tiles_radius>
  ///   <tile_mesh_size>200</tile_mesh_size>
  /// </plugin>
  /// \endverbatim
  class WaterVisual : public System,
                      public ISystemConfigure,
                      public ISystemPreUpdate
  {
    /// \brief Constructor.
    public: WaterVisual();
    /// \brief Destructor.
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
