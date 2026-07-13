/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef GZ_SIM_WAVES_FFTWAVESIMULATION_HH_
#define GZ_SIM_WAVES_FFTWAVESIMULATION_HH_

#include <cstddef>
#include <cstdint>
#include <memory>

#include <Eigen/Dense>

#include "gz/sim/waves/WaveSimulation.hh"

namespace gz::sim::waves
{

struct WaveParameters;

/// \brief Stochastic FFT-based wave field engine backed by the Apache-2.0 EncinoWaves
/// spectral library (Horvath 2015): the inverse 2D FFT of an empirically
/// modelled directional spectrum (TMA/JONSWAP/PM + directional spreading +
/// dispersion), selected via the <spectrum>/<spreading>/<dispersion> params.
///
/// The output is a periodic tile of size `tileSize × tileSize` metres.
/// Queries outside the tile are wrapped via `fmod`. Each call to `Update`
/// regenerates the grid for that time; per-point queries (`Elevation`,
/// `ParticleVelocity`, ...) bilinear-sample the stored grid.
///
/// \note Sim time is handed to EncinoWaves in single precision (its API is
/// the float instantiation), so on multi-hour runs the float grid coarsens
/// and gradually degrades the wave animation and the particle-velocity
/// finite difference.
class FFTWaveSimulation final : public IWaveField
{
  /// \brief Default-construct an unconfigured field. Call `SetParameters`
  /// before `Update`/sampling. Used by the engine factory (`MakeFFTWaveField`).
  public: FFTWaveSimulation();

  /// \brief Construct from spectrum / wind parameters.
  /// \param[in] _params Wave parameters; derives wind speed from `period`
  ///   (deep-water PMS relation: V19 ≈ 0.879·g/omegaP). `gain` scales
  ///   spectrum amplitudes uniformly. `direction` is parsed but not yet
  ///   applied (EncinoWaves assumes wind along +X).
  /// \param[in] _tileSize Physical tile extent in metres; the wave field is
  ///   periodic with this period along both x and y.
  /// \param[in] _gridSize Resolution per axis (must be a power of two for
  ///   the FFT path; 64, 128, 256 typical).
  /// \param[in] _seed RNG seed for the Gaussian-distributed amplitudes; same
  ///   seed → same wave field bit-for-bit across runs.
  public: FFTWaveSimulation(const WaveParameters &_params,
                            double _tileSize,
                            std::size_t _gridSize,
                            std::uint32_t _seed);

  /// \brief Destructor (out-of-line so the EncinoState pimpl stays in the .cc).
  public: ~FFTWaveSimulation() override;

  // Documentation inherited
  public: void SetParameters(const WaveParameters &_params) override;
  // Documentation inherited
  public: double Elevation(double _x, double _y, double _t) const override;
  // Documentation inherited
  public: gz::math::Vector3d ParticleVelocity(
    double _x, double _y, double _t) const override;
  // Documentation inherited
  public: gz::math::Vector3d Normal(
    double _x, double _y, double _t) const override;
  // Documentation inherited
  public: double Jacobian(double _x, double _y, double _t) const override;
  // Documentation inherited
  public: void Update(double _simTime) override;
  // Documentation inherited
  public: std::string_view Kind() const override { return "fft"; }
  // Documentation inherited
  public: std::optional<TileSize> Bounds() const override
  {
    return TileSize{this->tileSize, this->tileSize};
  }
  // Documentation inherited
  public: const WaveField2D *Field() const override;

  // ---- Accessors for unit tests + visual heightmap upload ----------------

  /// \brief Grid resolution per axis (power of two).
  public: std::size_t GridSize() const { return this->gridSize; }
  /// \brief Physical tile extent per axis [m]; the field is periodic with it.
  public: double TileSizeMeters() const { return this->tileSize; }

  /// \brief Surface elevation field η(x, y, t), refreshed by Update().
  public: const Eigen::MatrixXd &HeightGrid() const
  {
    return this->heightGrid;
  }
  /// \brief Horizontal x-displacement field Dx(x, y, t), refreshed by Update().
  /// Multiplied by a "choppiness" factor in the visual shader to sharpen
  /// crests (Tessendorf 2001, eq. 29). Same grid layout as `HeightGrid()`.
  public: const Eigen::MatrixXd &DispXGrid() const
  {
    return this->dispXGrid;
  }
  /// \brief Horizontal y-displacement field Dy(x, y, t).
  public: const Eigen::MatrixXd &DispYGrid() const
  {
    return this->dispYGrid;
  }

  /// \brief Bilinear sample of `_grid` at the given world (x, y), wrapping
  /// queries outside the tile to its periodic image.
  /// \param[in] _grid The grid to sample.
  /// \param[in] _x    World x coordinate [m].
  /// \param[in] _y    World y coordinate [m].
  /// \return The bilinearly-interpolated grid value.
  private: double BilinearSample(const Eigen::MatrixXd &_grid,
                                 double _x, double _y) const;

  // Configuration. Default member-inits keep a default-constructed instance
  // benign until SetParameters() runs (the factory path).

  /// \brief Physical tile extent per axis [m].
  private: double tileSize{200.0};
  /// \brief Grid resolution per axis (power of two).
  private: std::size_t gridSize{128};
  /// \brief Wind speed [m/s], derived from `period`.
  private: double windSpeed{0.0};
  /// \brief Spectrum amplitude gain.
  private: double gain{1.0};
  /// \brief Startup-ramp time constant τ [s].
  private: double tau{2.0};
  /// \brief Tessendorf choppiness multiplier applied to Dx/Dy in Update():
  /// WaveField2D carries the FINAL horizontal displacement, so the engine —
  /// not the shader — owns this scaling. Negative pinches crests.
  private: double choppiness{-1.0};

  /// \brief Surface elevation grid η (real domain), regenerated by Update().
  private: Eigen::MatrixXd heightGrid;
  /// \brief Horizontal x-displacement grid Dx (Tessendorf eq. 29); used by the
  /// visual shader for choppy, asymmetric crests.
  private: Eigen::MatrixXd dispXGrid;
  /// \brief Horizontal y-displacement grid Dy.
  private: Eigen::MatrixXd dispYGrid;
  /// \brief Folding/whitecap metric (Encino path): per-cell minimum eigenvalue
  /// of the displacement Jacobian, 1 = flat. Sampled by Jacobian() → FoamMask().
  private: Eigen::MatrixXd minEGrid;

  /// \brief Water-particle velocity grids [m/s] — the Eulerian time derivative
  /// of the displacement field (∂Dx/∂t, ∂Dy/∂t, ∂η/∂t). Filled lazily by the
  /// first ParticleVelocity() call after an Update (finite difference of a
  /// scratch propagation at t+dt), so consumers that never query velocity —
  /// notably the renderer, which reads Field() only — skip the second
  /// propagation entirely. Mutable: the lazy fill happens inside the const
  /// query and relies on the same external serialisation as Update().
  private: mutable Eigen::MatrixXd velXGrid;
  /// \brief Water-particle velocity grid, y component [m/s].
  private: mutable Eigen::MatrixXd velYGrid;
  /// \brief Water-particle velocity grid, z (vertical) component [m/s].
  private: mutable Eigen::MatrixXd velZGrid;
  /// \brief Sim time the velocity grids were computed for. Velocity is stale
  /// (recomputed on the next query) whenever this differs from lastUpdateT;
  /// SetParameters resets it so a reconfigure never serves old-recipe grids.
  private: mutable double velTime{-1.0};

  /// \brief Column-major view into the grids above, returned by Field() as the
  /// engine-agnostic rendering contract. Repopulated on each call from the
  /// current grid data() — Update reassigns the grids, so a cached pointer
  /// would dangle. Mutable because Field() is const.
  private: mutable WaveField2D field;

  /// \brief Forward declaration of the EncinoWaves Update state. Defined
  /// entirely in FFTWaveSimulation.cc so the vendored EncinoWaves headers don't
  /// leak into this public include surface.
  private: struct EncinoState;
  /// \brief The EncinoWaves-backed Update state (pimpl).
  private: std::unique_ptr<EncinoState> encino;

  /// \brief Physics-based amplitude calibration for the Encino path. Encino's
  /// intrinsic field variance is far larger than a physical sea state at our
  /// wind speeds, so we measure its intrinsic RMS once at construction and
  /// store the factor that rescales the significant wave height to the
  /// fully-developed Pierson-Moskowitz relation (Hs = 0.21·V19.5²/g). Applied
  /// to η/Dx/Dy every Update.
  private: double encinoScale{1.0};

  /// \brief Sim time of the last Update(). The field is deterministic in time,
  /// so Update() short-circuits on a repeat call — letting several consumers
  /// (the source system plus each WaveBuoyancy that advances its own instance)
  /// converge on one FFT per tick. -1 = never updated (sim time is always ≥ 0).
  private: double lastUpdateT{-1.0};
};

/// \brief Factory: a default-constructed FFT wave-field engine (apply
/// `SetParameters` before use). Registered under the "fft" token so
/// `CreateWaveSimulation` can rebuild the engine from a serialized `Wavefield`
/// component on the GUI side.
std::shared_ptr<IWaveField> MakeFFTWaveField();

}  // namespace gz::sim::waves

#endif  // GZ_SIM_WAVES_FFTWAVESIMULATION_HH_
