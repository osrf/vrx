/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <Eigen/Dense>

#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>
// Pull in the gz-sim "systems" sub-namespace declared inside the
// `inline namespace v10`. Without this, our `namespace gz::sim::systems`
// below resolves to a *different* namespace (no v10) than the one
// WaterVisual sees, producing mismatched symbol mangling at link time.
#include <gz/sim/System.hh>

namespace gz::sim::systems
{
  /// \brief A CPU→GPU heightmap texture used by the FFT visual path.
  ///
  /// Owns an Ogre Next `TextureGpu` of single-channel 32-bit float pixels
  /// (`PFG_R32_FLOAT`), `gridSize × gridSize`. Each frame, the CPU-side
  /// FFT simulation writes the latest height field into a staging texture,
  /// which is then asynchronously uploaded to the GPU texture and bound to
  /// the material's "heightMap" sampler.
  ///
  /// All Ogre Next interaction is encapsulated here so the rest of the
  /// plugin (`WaterVisual`) talks in terms of `Eigen::MatrixXd`.
  class HeightMapTexture
  {
  public:
    /// \brief Construct a heightmap of the given resolution and bind it to
    /// the specified material's "heightMap" sampler.
    /// \param[in] _scene The (Ogre2) scene that owns the rendering pipeline.
    /// \param[in] _material The material whose pass to bind to. The material
    ///   is expected to have a fragment/vertex shader with a
    ///   `uniform sampler2D heightMap`.
    /// \param[in] _gridSize Resolution per axis (power of two).
    /// \param[in] _textureName Unique name for the GPU texture.
    HeightMapTexture(gz::rendering::ScenePtr _scene,
                     gz::rendering::MaterialPtr _material,
                     std::size_t _gridSize,
                     const std::string &_textureName);

    /// \brief Destructor — releases the TextureGpu.
    ~HeightMapTexture();

    /// \brief Upload the supplied height grid to the GPU. Expects an
    /// `gridSize × gridSize` matrix of `double` heights.
    /// \return True on success, false if the upload couldn't proceed (e.g.
    ///   texture not yet resident, or unexpected grid size).
    bool Upload(const Eigen::MatrixXd &_grid);

    /// \brief True once the texture is GPU-resident and bound.
    bool Ready() const { return this->ready_; }

  private:
    /// \brief Implementation detail kept out of the header to avoid leaking
    /// Ogre Next types into every translation unit that includes us.
    class Impl;
    std::unique_ptr<Impl> impl_;

    std::size_t gridSize_;
    bool ready_{false};
  };
}  // namespace gz::sim::systems

#endif  // GZ_SIM_SYSTEMS_HEIGHTMAPTEXTURE_HH_
