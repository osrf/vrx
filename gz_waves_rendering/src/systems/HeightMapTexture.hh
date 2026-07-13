/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef GZ_SIM_SYSTEMS_HEIGHTMAPTEXTURE_HH_
#define GZ_SIM_SYSTEMS_HEIGHTMAPTEXTURE_HH_

#include <cstddef>
#include <string>

#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>
// Pull in the gz-sim "systems" sub-namespace declared inside the
// `inline namespace v10`. Without this, our `namespace gz::sim::systems`
// below resolves to a *different* namespace (no v10) than the one
// WaterVisual sees, producing mismatched symbol mangling at link time.
#include <gz/sim/System.hh>

namespace gz::sim::systems
{
  /// \brief A CPU→GPU heightmap texture used by the water visual path.
  ///
  /// Owns an Ogre Next `TextureGpu` of four-channel 32-bit float pixels
  /// (`PFG_RGBA32_FLOAT`), `gridSize × gridSize`, packing (η, Dx, Dy, foam)
  /// per texel. Each frame, the CPU-side wave engine's grids are written
  /// into a staging texture, asynchronously uploaded to the GPU texture, and
  /// bound to the material's "heightMap" sampler.
  ///
  /// All Ogre Next interaction is encapsulated here so the rest of the
  /// plugin (`WaterVisual`) talks in terms of the raw `WaveField2D` grids.
  class HeightMapTexture
  {
    /// \brief Construct a heightmap of the given resolution and bind it to
    /// the specified material's "heightMap" sampler.
    /// \param[in] _scene The (Ogre2) scene that owns the rendering pipeline.
    /// \param[in] _material The material whose pass to bind to. The material
    ///   is expected to have a fragment/vertex shader with a
    ///   `uniform sampler2D heightMap`.
    /// \param[in] _gridSize Resolution per axis (power of two).
    /// \param[in] _textureName Unique name for the GPU texture.
    public: HeightMapTexture(const gz::rendering::ScenePtr &_scene,
                     const gz::rendering::MaterialPtr &_material,
                     std::size_t _gridSize,
                     const std::string &_textureName);

    /// \brief Destructor — releases the TextureGpu.
    public: ~HeightMapTexture();

    /// \brief Upload the supplied height + horizontal-displacement grids to
    /// the GPU. All buffers are column-major `_n × _n` — the `WaveField2D`
    /// layout, element (i, j) at index `i + j*_n` — and are handed to the
    /// bridge unchanged, which packs them into a single RGBA32F texture
    /// (η, Dx, Dy, foam) oriented so texel (u, v) = grid (x_u, y_v), as the
    /// water vertex/fragment shaders sample it.
    /// \param[in] _eta   Surface elevation grid → R channel (required).
    /// \param[in] _dispX Horizontal x-displacement grid → G channel; null ⇒ 0.
    /// \param[in] _dispY Horizontal y-displacement grid → B channel; null ⇒ 0.
    /// \param[in] _foam  Optional per-cell folding / foam metric → the
    ///   texture's alpha channel; null leaves alpha at zero.
    /// \param[in] _n     Grid resolution per axis (must equal `gridSize`).
    /// \return True on success, false if the upload couldn't proceed (e.g.
    ///   null elevation, texture not yet resident, or unexpected grid size).
    public: bool Upload(const double *_eta, const double *_dispX,
                const double *_dispY, const double *_foam,
                std::size_t _n);

    /// \brief Patch a named tex unit on the bound material to use
    /// anisotropic trilinear filtering. Needed for dense bumpmaps so
    /// they don't alias at distance.
    /// \param[in] _texUnitName The texture unit to patch.
    /// \return True if the unit was found and patched.
    public: bool SetTexFiltering(const std::string &_texUnitName);

    /// \brief True once the texture is GPU-resident and bound.
    /// \return Whether the texture is ready.
    public: bool Ready() const { return this->ready; }

    /// \brief Resolution per axis this texture was created with. Upload()
    /// only accepts grids of exactly this size, so a consumer whose grid
    /// resolution changed must create a new instance.
    public: std::size_t GridSize() const { return this->gridSize; }

    /// \brief Implementation detail kept out of the header to avoid leaking
    /// Ogre Next types into every translation unit that includes us.
    private: class Impl;
    /// \brief Pimpl owning the Ogre Next texture/staging state.
    private: std::unique_ptr<Impl> impl;

    /// \brief Grid resolution per axis.
    private: std::size_t gridSize;
    /// \brief True once the texture is GPU-resident and bound.
    private: bool ready{false};
  };
}  // namespace gz::sim::systems

#endif  // GZ_SIM_SYSTEMS_HEIGHTMAPTEXTURE_HH_
