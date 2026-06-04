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

    /// \brief Upload the supplied height + horizontal-displacement grids to
    /// the GPU. All matrices must be `gridSize × gridSize`. They are packed
    /// into a single RGBA32F texture (η, Dx, Dy, foam) and consumed by the
    /// FFT vertex/fragment shaders.
    /// \param[in] _foam Optional per-cell folding / foam metric → the
    ///   texture's alpha channel; null leaves alpha at zero.
    /// \return True on success, false if the upload couldn't proceed (e.g.
    ///   texture not yet resident, or unexpected grid size).
    bool Upload(const Eigen::MatrixXd &_eta,
                const Eigen::MatrixXd &_dispX,
                const Eigen::MatrixXd &_dispY,
                const Eigen::MatrixXd *_foam = nullptr);

    /// \brief Upload the per-cell slope grid (∂η/∂x, ∂η/∂y) to a
    /// dedicated `slopeMap` texture. Bound to the visual material on
    /// first call. The VS reads N = normalize(-slope.x, -slope.y, 1)
    /// per-vertex for spectrum-accurate surface normals.
    bool UploadSlope(const Eigen::MatrixXd &_slopeX,
                     const Eigen::MatrixXd &_slopeY);

    /// \brief Upload the per-cell chop-derivative grids (∂Dx/∂x,
    /// ∂Dy/∂y, ∂Dx/∂y) to a dedicated `chopDerivMap` texture. With
    /// the slopeMap this completes the five derivative grids needed
    /// for the full Tessendorf chop-aware tangent + normal.
    bool UploadChopDerivatives(const Eigen::MatrixXd &_dDxDx,
                                const Eigen::MatrixXd &_dDyDy,
                                const Eigen::MatrixXd &_dDxDy);

    /// \brief GPU-FFT path: dispatch a compute shader that writes the
    /// heightmap texture directly on the GPU, replacing the CPU
    /// `Upload(...)` call. The bridge lazily sets up the
    /// `HlmsComputeJob` on the first invocation.
    /// \param _shaderAbsPath Absolute path to the `.glsl` compute shader.
    /// \param _simTimeS Current simulation time [s].
    /// \param _tileSizeM Physical tile extent [m].
    /// \return True if the dispatch was issued.
    bool Dispatch(const std::string &_shaderAbsPath,
                  float _simTimeS,
                  float _tileSizeM);

    /// \brief Stage 6: replace the gz::rendering Visual material with a
    /// procedurally-built `Ogre::Item` rendered through an
    /// `HlmsPbsDatablock`. Bypasses `Visual::SetMaterial` AND
    /// `HlmsLowLevel`, which are together the source of the 2-min
    /// first-frame stall on Jetty + NVIDIA Blackwell. Step 6.0 produces
    /// a plain colored plane; step 6.1+ adds the custom-piece hook for
    /// heightmap-driven vertex displacement.
    bool CreatePbsVisual(double _planeSizeM, int _planeSegments,
                         double _wx, double _wy, double _wz,
                         const std::string &_name);

    /// \brief Stage 2 of the GPU-FFT plan: upload the time-invariant
    /// Phillips spectrum to a persistent GPU texture. Called once
    /// after creation; reads from `FFTWaveSimulation::H0`/`H0Conj`.
    /// Sizes must equal `_gridSize × _gridSize` row-major doubles.
    /// ω(k) is computed in-shader.
    bool UploadSpectrum(const double *_h0Re, const double *_h0Im,
                        const double *_h0ConjRe, const double *_h0ConjIm,
                        int _gridSize);

    /// \brief Stage 2 dispatch: evolve the spectrum to `h(k, t)` on the
    /// GPU. Must follow `UploadSpectrum`. Output packs (η_hat, Dx_hat).
    bool EvolveDispatch(const std::string &_shaderAbsPath,
                        float _simTimeS, float _tauS = 0.0f,
                        float _tileSizeM = 200.0f);

    /// \brief Stage 2 dispatch (Dy chop): time-evolve the Dy
    /// displacement spectrum into a dedicated `hktTexDy`.
    bool EvolveDyDispatch(const std::string &_shaderAbsPath,
                          float _simTimeS, float _tauS = 0.0f,
                          float _tileSizeM = 200.0f);

    /// \brief Stage 4 dispatch: combine the η+Dx IFFT output and the
    /// Dy IFFT output into the final RGBA texture (η, Dx, Dy, _)
    /// bound to the visual material. Two passes are needed because
    /// OgreNext's OpenGL compute can only bind one texture sampler
    /// reliably per dispatch. Idempotent; call once per frame after
    /// `IfftDispatch`.
    bool CombineDispatch(const std::string &_etaDxShaderAbsPath,
                          const std::string &_dyShaderAbsPath);

    /// \brief Stage 3 dispatch: run the GPU IFFT (Cooley-Tukey radix-2)
    /// on the latest `h(k, t)` texture produced by EvolveDispatch. The
    /// result is the spatial wave height field η(x, y, t). Must follow
    /// EvolveDispatch each frame. Two shaders are required:
    /// \param[in] _bitrevShaderAbsPath bit-reversal permutation pass.
    /// \param[in] _butterShaderAbsPath butterfly stage (parameterised
    ///   by axis + stage).
    bool IfftDispatch(const std::string &_bitrevShaderAbsPath,
                      const std::string &_butterShaderAbsPath);

    /// \brief Diagnostic: brute-force O(N²) 2D IFFT in one dispatch.
    /// Used to verify whether the radix-2 Cooley-Tukey pipeline's
    /// output matches a known-correct reference implementation. Gated
    /// by `GZ_WAVES_GPU_FFT_NAIVE=1`.
    bool IfftNaiveDispatch(const std::string &_naiveShaderAbsPath);

    /// \brief Diagnostic: upload CPU's (eta, dx, dy) grids straight
    /// into the IFFT output texture, bypassing evolve+IFFT. Used to
    /// isolate "is our compute chain wrong" vs "is our visual sampling
    /// wrong" — if the visual then matches CPU exactly, the compute
    /// chain is what's wrong.
    bool CpuFeed(const Eigen::MatrixXd &_eta,
                 const Eigen::MatrixXd &_dispX,
                 const Eigen::MatrixXd &_dispY);

    /// \brief Stage 4: true once the visual material's "heightMap"
    /// sampler has been swapped to the GPU IFFT output. Until then
    /// the CPU `Upload(...)` path must continue running so the
    /// material has *something* sensible to sample.
    bool GpuOutputBound() const;

    /// \brief Diagnostic: dispatch a compute shader that overwrites
    /// the IFFT output texture with a known sine pattern. Used to
    /// isolate binding/sampling bugs from compute bugs. Gate it with
    /// the `GZ_WAVES_GPU_FFT_TEST_PATTERN=1` env var so it's a no-op
    /// in production. Must follow `IfftDispatch` (so ifftFinalTex
    /// exists).
    bool TestPatternDispatch(const std::string &_shaderAbsPath,
                             float _simTimeS, float _amplitude);

    /// \brief Diagnostic: dispatch a compute shader that views the
    /// hkt texture (Stage 2 evolve's output) by copying a scaled
    /// magnitude into the IFFT output texture. Used to distinguish
    /// "evolve produces nothing" from "IFFT mangles evolve's output".
    /// Gate with `GZ_WAVES_GPU_FFT_VIEW_HKT=1`.
    bool ViewHktDispatch(const std::string &_shaderAbsPath, float _scale);

    /// \brief Diagnostic: async readback of one cell of `h0Tex` for
    /// GPU↔CPU value comparison. Returns the four floats stored at
    /// pixel (col=_j, row=_i) of h0Tex — i.e. CPU's
    /// (h0(_i, _j).real, h0(_i, _j).imag,
    ///  h0Conj(_i, _j).real, h0Conj(_i, _j).imag).
    bool ReadbackH0Cell(int _i, int _j,
                        float *_outRe, float *_outIm,
                        float *_outConjRe, float *_outConjIm) const;

    /// \brief Diagnostic: async readback of η at one cell of
    /// `ifftFinalTex` (the Stage 3 IFFT output that the visual
    /// samples). Compare to CPU's heightGrid_(i, j) at the same t.
    bool ReadbackIfftCell(int _i, int _j, float *_outEta) const;

    /// \brief Diagnostic: async readback of h(k, t) at one cell of
    /// `hktTex` (the Stage 2 evolve output). Compare to CPU's
    /// freshly-computed h(k, t) at the same t.
    bool ReadbackHktCell(int _i, int _j,
                         float *_outRe, float *_outIm) const;

    /// \brief Diagnostic: scan `combinedTex` for non-finite (NaN/Inf)
    /// cells and report per-channel ranges. `outBadCount` returns the
    /// number of cells where any channel is non-finite. If at least
    /// one such cell exists, `outFirstBadI/J` and `outFirstBadRgba`
    /// are populated with the first bad cell's coordinates + values.
    /// `outMinRgba`/`outMaxRgba` summarise the finite cells.
    bool ReadbackCombinedScan(int *_outBadCount,
                              float *_outMinRgba, float *_outMaxRgba,
                              int *_outFirstBadI, int *_outFirstBadJ,
                              float *_outFirstBadRgba) const;

    /// \brief Patch a named tex unit on the bound material to use
    /// anisotropic trilinear filtering. Needed for dense bumpmaps so
    /// they don't alias at distance.
    bool SetTexFiltering(const std::string &_texUnitName);

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
