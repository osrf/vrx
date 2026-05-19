/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef GZ_SIM_WAVES_FFTWAVESIMULATION_HH_
#define GZ_SIM_WAVES_FFTWAVESIMULATION_HH_

#include <complex>
#include <cstddef>
#include <cstdint>

#include <Eigen/Dense>

#include "gz/sim/waves/WaveSimulation.hh"

namespace gz::sim::waves
{

struct WaveParameters;

/// \brief Stochastic FFT-based wave model following Tessendorf (2001).
/// Synthesizes a wave field as the inverse 2D FFT of a Phillips-spectrum
/// amplitude grid evolved by deep-water dispersion. CPU only — uses
/// `Eigen::FFT` (KissFFT backend by default) so we don't depend on FFTW3.
///
/// The output is a periodic tile of size `tileSize × tileSize` metres.
/// Queries outside the tile are wrapped via `fmod`. Each call to `Update`
/// regenerates the grid for that time; per-point queries (`Elevation`,
/// `ParticleVelocity`, ...) bilinear-sample the stored grid.
class FFTWaveSimulation : public IWaveSimulation
{
public:
  /// \brief Construct from spectrum / wind parameters.
  /// \param[in] _params Wave parameters; uses `direction` as the wind heading
  ///   and derives wind speed from `period` (deep-water PMS relation:
  ///   V19 ≈ 0.879·g/omegaP). `gain` scales spectrum amplitudes uniformly.
  /// \param[in] _tileSize Physical tile extent in metres; the wave field is
  ///   periodic with this period along both x and y.
  /// \param[in] _gridSize Resolution per axis (must be a power of two for
  ///   the FFT path; 64, 128, 256 typical).
  /// \param[in] _seed RNG seed for the Gaussian-distributed amplitudes; same
  ///   seed → same wave field bit-for-bit across runs.
  FFTWaveSimulation(const WaveParameters &_params,
                    double _tileSize,
                    std::size_t _gridSize,
                    std::uint32_t _seed);

  ~FFTWaveSimulation() override = default;

  // IWaveSimulation
  double Elevation(double x, double y, double t) const override;
  gz::math::Vector3d ParticleVelocity(
    double x, double y, double t) const override;
  gz::math::Vector3d Normal(double x, double y, double t) const override;
  double Jacobian(double x, double y, double t) const override;
  void Update(double simTime) override;
  std::string_view Kind() const override { return "fft"; }
  std::optional<TileSize> Bounds() const override
  {
    return TileSize{this->tileSize_, this->tileSize_};
  }

  // ---- Accessors for unit tests + visual heightmap upload ----------------

  std::size_t GridSize() const { return this->gridSize_; }
  double TileSizeMeters() const { return this->tileSize_; }
  const Eigen::MatrixXd &HeightGrid() const { return this->heightGrid_; }
  /// \brief Horizontal x-displacement field Dx(x, y, t), refreshed by Update().
  /// Multiplied by a "choppiness" factor in the visual shader to sharpen
  /// crests (Tessendorf 2001, eq. 29). Same grid layout as `HeightGrid()`.
  const Eigen::MatrixXd &DispXGrid() const { return this->dispXGrid_; }
  /// \brief Horizontal y-displacement field Dy(x, y, t).
  const Eigen::MatrixXd &DispYGrid() const { return this->dispYGrid_; }
  /// \brief Slope ∂η/∂x at each grid cell. Used by the visual to read
  /// per-vertex surface normals from a texture instead of computing
  /// them via finite differences (smoother and more accurate at the
  /// short wavelengths the spectrum carries).
  const Eigen::MatrixXd &SlopeXGrid() const { return this->slopeXGrid_; }
  /// \brief Slope ∂η/∂y at each grid cell.
  const Eigen::MatrixXd &SlopeYGrid() const { return this->slopeYGrid_; }
  /// \brief Chop displacement derivative ∂Dx/∂x.
  const Eigen::MatrixXd &DispDxDxGrid() const { return this->dispDxDxGrid_; }
  /// \brief Chop displacement derivative ∂Dy/∂y.
  const Eigen::MatrixXd &DispDyDyGrid() const { return this->dispDyDyGrid_; }
  /// \brief Chop displacement cross derivative ∂Dx/∂y (= ∂Dy/∂x).
  const Eigen::MatrixXd &DispDxDyGrid() const { return this->dispDxDyGrid_; }

  /// \brief Time-invariant Phillips spectrum amplitudes `h0(k)`. Used by
  /// the GPU-FFT path (`docs/waves_gpu_fft_plan.md`, Stage 2) to upload
  /// the spectrum to a GPU texture once at init; the GPU evolve compute
  /// shader then computes `h(k, t)` each frame without re-running the
  /// random sampling and Phillips evaluation.
  const Eigen::MatrixXcd &H0() const     { return this->h0_; }
  /// \brief Conjugate spectrum amplitudes `conj(h0(-k))`.
  const Eigen::MatrixXcd &H0Conj() const { return this->h0Conj_; }
  /// \brief Cached dispersion frequencies `omega(k) = sqrt(g·|k|)`.
  const Eigen::MatrixXd &OmegaGrid() const { return this->omegaGrid_; }

private:
  /// \brief Phillips spectrum P_h(k) (Tessendorf 2001, eq. 23).
  double Phillips(double kx, double ky) const;

  /// \brief Bilinear sample of `grid` at the given world (x, y), wrapping
  /// queries outside the tile to its periodic image.
  double BilinearSample(const Eigen::MatrixXd &grid,
                        double x, double y) const;

  /// \brief Startup ramp factor `(1 - exp(-t/tau))`, clamped to [0, 1].
  double Ramp(double t) const;

  // Configuration
  double        tileSize_;
  std::size_t   gridSize_;
  double        windSpeed_;
  double        windDirX_;
  double        windDirY_;
  double        gain_;
  double        tau_;

  // Base spectrum amplitudes h0(k), generated once
  Eigen::MatrixXcd h0_;
  Eigen::MatrixXcd h0Conj_;

  // Per-update height grid (real domain), regenerated by Update(t)
  Eigen::MatrixXd heightGrid_;
  // Per-update horizontal displacement grids (Tessendorf eq. 29). Used by
  // the visual shader to produce choppy, asymmetric wave crests.
  Eigen::MatrixXd dispXGrid_;
  Eigen::MatrixXd dispYGrid_;
  // Per-update slope grids: ∂η/∂x and ∂η/∂y at each cell. Combined
  // gives the surface normal as normalize(-∂η/∂x, -∂η/∂y, 1).
  Eigen::MatrixXd slopeXGrid_;
  Eigen::MatrixXd slopeYGrid_;
  // Per-update chop-displacement derivative grids. Together with the
  // slope grids they let the visual VS compute the full chop-aware
  // Tessendorf tangent + normal at each vertex.
  Eigen::MatrixXd dispDxDxGrid_;
  Eigen::MatrixXd dispDyDyGrid_;
  Eigen::MatrixXd dispDxDyGrid_;

  // Per-axis wavenumber arrays kx[i], ky[j] (precomputed)
  Eigen::VectorXd kxRow_;
  Eigen::VectorXd kyCol_;

  // Per-update dispersion factor omega(k) (precomputed)
  Eigen::MatrixXd omegaGrid_;
};

}  // namespace gz::sim::waves

#endif  // GZ_SIM_WAVES_FFTWAVESIMULATION_HH_
