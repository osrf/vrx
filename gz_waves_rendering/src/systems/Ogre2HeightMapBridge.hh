/*
 * Copyright (C) 2026 Honu Robotics
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
  /// \brief Opaque handle to a per-material heightmap instance.
  typedef void *waves_heightmap_t;

  /// \brief Create a heightmap and bind it to the material as a "heightMap"
  /// texture unit. Returns NULL on failure.
  /// \param scene  Raw pointer to a `gz::rendering::Scene` (must be Ogre2).
  /// \param material  Raw pointer to a `gz::rendering::Material`.
  /// \param grid_size  Texture resolution per axis (power of two).
  /// \param name  Unique texture name for Ogre Next's bookkeeping.
  waves_heightmap_t waves_ogre2_heightmap_create(
      void *_scene,
      void *_material,
      std::size_t _grid_size,
      const char *_name);

  /// \brief Upload three `rows × cols` row-major double matrices (height,
  /// horizontal x-displacement, horizontal y-displacement) into the RGB
  /// channels of the RGBA32F heightmap texture. `foam_grid`, when non-null,
  /// fills the alpha channel with a per-cell folding / foam metric (the
  /// displacement Jacobian's minimum eigenvalue; 1 = flat, < 1 → folding);
  /// pass null to leave alpha zero. Each non-null pointer must reference at
  /// least `rows * cols` doubles in row-major order. Returns 1 on success,
  /// 0 on failure (e.g. texture not yet resident).
  int waves_ogre2_heightmap_upload(
      waves_heightmap_t _handle,
      const double *_eta_grid,
      const double *_disp_x_grid,
      const double *_disp_y_grid,
      const double *_foam_grid,
      int _rows,
      int _cols);

  /// \brief 1 if the GPU texture is GPU-resident and the material has been bound,
  /// 0 otherwise.
  int waves_ogre2_heightmap_ready(waves_heightmap_t _handle);


  /// \brief Patch the samplerblock of a named tex unit on the bound material
  /// to use trilinear + anisotropic filtering. Used after the
  /// gz::rendering ShaderParam path has set a texture (which only
  /// installs default bilinear-without-mipmap filtering) — without
  /// this, dense bumpmap tilings alias badly at distance. Returns 1
  /// if the unit was found and patched.
  int waves_ogre2_heightmap_set_tex_filtering(
      waves_heightmap_t _handle,
      const char *_tex_unit_name);

  /// \brief Release the heightmap. Safe on NULL.
  void waves_ogre2_heightmap_destroy(waves_heightmap_t _handle);
}

#endif  // GZ_SIM_SYSTEMS_OGRE2HEIGHTMAPBRIDGE_HH_
