/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#ifndef GZ_SIM_SYSTEMS_OGRE2HEIGHTMAPBRIDGE_HH_
#define GZ_SIM_SYSTEMS_OGRE2HEIGHTMAPBRIDGE_HH_

// C ABI for the WaterVisual plugin to talk to Ogre Next without itself
// linking against libgz-rendering-ogre2.so. The bridge lives in
// libwaves-ogre2-bridge.so and is loaded by WaterVisual via dlopen() the
// first time the FFT visual path is requested.
//
// All inputs are opaque pointers; the bridge knows the real types because
// it owns the Ogre Next headers + symbols. Lifetimes are managed by the
// caller (the gz::rendering::Scene/Material remain valid for the lifetime
// of the heightmap handle).

#include <cstddef>

extern "C"
{
  /// Opaque handle to a per-material heightmap instance.
  typedef void *waves_heightmap_t;

  /// Create a heightmap and bind it to the material as a "heightMap"
  /// texture unit. Returns NULL on failure.
  /// \param scene  Raw pointer to a `gz::rendering::Scene` (must be Ogre2).
  /// \param material  Raw pointer to a `gz::rendering::Material`.
  /// \param grid_size  Texture resolution per axis (power of two).
  /// \param name  Unique texture name for Ogre Next's bookkeeping.
  waves_heightmap_t waves_ogre2_heightmap_create(
      void *scene,
      void *material,
      std::size_t grid_size,
      const char *name);

  /// Upload three `rows × cols` row-major double matrices (height,
  /// horizontal x-displacement, horizontal y-displacement) into the
  /// RGBA32F heightmap texture. The alpha channel is reserved for a future
  /// foam/Jacobian mask and is currently written as zero. Each pointer
  /// must reference at least `rows * cols` doubles in row-major order.
  /// Returns 1 on success, 0 on failure (e.g. texture not yet resident).
  int waves_ogre2_heightmap_upload(
      waves_heightmap_t handle,
      const double *eta_grid,
      const double *disp_x_grid,
      const double *disp_y_grid,
      int rows,
      int cols);

  /// 1 if the GPU texture is GPU-resident and the material has been bound,
  /// 0 otherwise.
  int waves_ogre2_heightmap_ready(waves_heightmap_t handle);

  /// GPU-FFT path. Dispatch a compute shader that writes the heightmap
  /// texture directly on the GPU. The bridge creates the
  /// `Ogre::HlmsComputeJob` on first call (loading `shader_abs_path` and
  /// registering its parent directory as a resource location), and
  /// reuses it on subsequent calls. Each invocation refreshes the
  /// `(t, tileSize, gridSize)` const buffer and dispatches.
  ///
  /// Stage 1 of `docs/waves_gpu_fft_plan.md`. Replaces the CPU
  /// `*_upload` path once the GPU pipeline is producing real wave data
  /// (Stages 2-3).
  ///
  /// \param handle  Heightmap handle returned by `_create`.
  /// \param shader_abs_path  Absolute path to a `.glsl` compute shader.
  ///   Its directory is registered as a "General" resource location on
  ///   first call.
  /// \param sim_time_s  Current simulation time [s]; uploaded as the `t`
  ///   uniform.
  /// \param tile_size_m  Physical tile extent passed to the shader.
  /// \return 1 on success, 0 on any failure (texture not resident,
  ///   shader compile failure, etc.).
  int waves_ogre2_heightmap_compute_dispatch(
      waves_heightmap_t handle,
      const char *shader_abs_path,
      float sim_time_s,
      float tile_size_m);

  /// Stage 6 (HlmsPbs migration). Build a procedural plane Ogre::Item,
  /// apply a fresh `HlmsPbsDatablock` to it via `SubItem::setDatablock`,
  /// and parent it under the scene root. Bypasses
  /// `gz::rendering::Visual::SetMaterial` AND `HlmsLowLevel` entirely —
  /// step 6.0 is the load-time experiment to verify HlmsPbs avoids the
  /// stall. Step 6.1 will hook a custom piece file to add vertex
  /// displacement from the heightmap.
  ///
  /// \param handle  Heightmap handle (only used for unique naming + the
  ///   eventual texture binding in step 6.1).
  /// \param plane_size_m  Side length of the rendered plane [m].
  /// \param plane_segments  Subdivision count per axis.
  /// \param world_x,y,z  World anchor for the plane.
  /// \param name  Unique base name for mesh/item/datablock/node.
  /// \return 1 on success.
  int waves_ogre2_heightmap_create_pbs_visual(
      waves_heightmap_t handle,
      double plane_size_m,
      int plane_segments,
      double world_x,
      double world_y,
      double world_z,
      const char *name);

  /// Stage 2 (Phillips spectrum on GPU). One-shot upload of the
  /// time-invariant Phillips spectrum (`h0`, `h0conj`) to a persistent
  /// GPU texture. Called once after the heightmap is created;
  /// subsequent `evolve_dispatch` calls read from this texture and
  /// compute ω(k)=sqrt(g·|k|) in-shader.
  /// \param handle Heightmap handle.
  /// \param h0_re,h0_im  N²-element row-major arrays of the
  ///   Phillips-spectrum amplitudes `h0(k)`.
  /// \param h0conj_re,h0conj_im  N²-element arrays of `conj(h0(-k))`.
  /// \param grid_size Side length N (must match the heightmap's).
  /// \return 1 on success, 0 on failure.
  int waves_ogre2_heightmap_upload_spectrum(
      waves_heightmap_t handle,
      const double *h0_re,
      const double *h0_im,
      const double *h0conj_re,
      const double *h0conj_im,
      int grid_size);

  /// Stage 2 dispatch: run the evolve compute shader to write the
  /// time-evolved spectrum h(k, t) into the bridge's `hktTex`. Lazily
  /// creates the HlmsComputeJob on first call, similar to
  /// `_compute_dispatch`.
  /// \param handle  Heightmap handle.
  /// \param shader_abs_path  Absolute path to `evolve.glsl`.
  /// \param sim_time_s  Current simulation time [s].
  /// \return 1 on success.
  int waves_ogre2_heightmap_evolve_dispatch(
      waves_heightmap_t handle,
      const char *shader_abs_path,
      float sim_time_s,
      float tau_s,
      float tile_size_m);

  /// Release the heightmap. Safe on NULL.
  void waves_ogre2_heightmap_destroy(waves_heightmap_t handle);
}

#endif  // GZ_SIM_SYSTEMS_OGRE2HEIGHTMAPBRIDGE_HH_
