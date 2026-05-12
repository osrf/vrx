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

  /// Upload an `rows × cols` matrix of double heights to the heightmap.
  /// The data is interpreted with `row_stride` doubles between successive
  /// rows in memory (Eigen column-major matrices set row_stride=1 and
  /// col_stride=rows; callers are expected to pass a row-major view).
  /// Returns 1 on success, 0 on failure (e.g. texture not yet resident).
  int waves_ogre2_heightmap_upload(
      waves_heightmap_t handle,
      const double *grid,
      int rows,
      int cols);

  /// 1 if the GPU texture is GPU-resident and the material has been bound,
  /// 0 otherwise.
  int waves_ogre2_heightmap_ready(waves_heightmap_t handle);

  /// Release the heightmap. Safe on NULL.
  void waves_ogre2_heightmap_destroy(waves_heightmap_t handle);
}

#endif  // GZ_SIM_SYSTEMS_OGRE2HEIGHTMAPBRIDGE_HH_
