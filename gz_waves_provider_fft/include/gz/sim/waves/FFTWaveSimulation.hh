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
#include <memory>

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
class FFTWaveSimulation : public IWaveField
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
  /// \brief Default-construct an unconfigured field. Call `SetParameters`
  /// before `Update`/sampling. Used by the gz-plugin provider loader.
  FFTWaveSimulation();

  FFTWaveSimulation(const WaveParameters &_params,
                    double _tileSize,
                    std::size_t _gridSize,
                    std::uint32_t _seed);

  ~FFTWaveSimulation() override;

  // IWaveField
  void SetParameters(const WaveParameters &_params) override;
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
  const WaveField2D *Field() const override;

  // ---- Accessors for unit tests + visual heightmap upload ----------------

  std::size_t GridSize() const { return this->gridSize_; }
  double TileSizeMeters() const { return this->tileSize_; }

  /// \brief True when this instance is driving Update(t) through the Apache-2.0
  /// Horvath spectrum library (TMA / JONSWAP + Hasselmann directional spread +
  /// chop-aware foam Jacobian) instead of the in-tree Phillips path. This is
  /// the default when the engine is built with EncinoWaves; Phillips is used
  /// only when encino isn't compiled in or the grid isn't a power of two. Slope
  /// and chop-derivative grids are NOT populated on the Encino path — visuals
  /// that need them should fall back to finite differences.
  bool UseEncino() const { return this->useEncino_; }

  /// \brief Enable or disable computation of the 5 derivative grids
  /// (∂η/∂x, ∂η/∂y, ∂Dx/∂x, ∂Dy/∂y, ∂Dx/∂y). Default true. When the
  /// visual won't upload them (Encino path, GPU-FFT path with
  /// useSlopeMap=0, or any consumer that finite-differences instead),
  /// disable to skip 5 of the 8 IFFTs per Update — substantial CPU
  /// savings on the Phillips path. Encino's Update branch already
  /// short-circuits before these are touched, so the flag is a no-op
  /// there.
  void SetComputeDerivatives(bool _on) { this->computeDerivatives_ = _on; }
  bool ComputeDerivatives() const { return this->computeDerivatives_; }

  const Eigen::MatrixXd &HeightGrid() const { return this->heightGrid_; }
  /// \brief Horizontal x-displacement field Dx(x, y, t), refreshed by Update().
  /// Multiplied by a "choppiness" factor in the visual shader to sharpen
  /// crests (Tessendorf 2001, eq. 29). Same grid layout as `HeightGrid()`.
  const Eigen::MatrixXd &DispXGrid() const { return this->dispXGrid_; }
  /// \brief Horizontal y-displacement field Dy(x, y, t).
  const Eigen::MatrixXd &DispYGrid() const { return this->dispYGrid_; }
  /// \brief Per-cell minimum eigenvalue of the displacement Jacobian (the
  /// folding / whitecap metric), 1 = flat, < 1 → folding. Populated only on
  /// the Encino path from its `MinE` output, rescaled to the calibrated
  /// amplitude. `Jacobian()` samples this; the in-tree Phillips path leaves it
  /// flat (1) — its foam is derived in the shader from the chop-deriv grids.
  const Eigen::MatrixXd &MinEGrid() const { return this->minEGrid_; }

private:
  /// \brief Phillips spectrum P_h(k) (Tessendorf 2001, eq. 23).
  double Phillips(double kx, double ky) const;

  /// \brief Bilinear sample of `grid` at the given world (x, y), wrapping
  /// queries outside the tile to its periodic image.
  double BilinearSample(const Eigen::MatrixXd &grid,
                        double x, double y) const;

  /// \brief Startup ramp factor `(1 - exp(-t/tau))`, clamped to [0, 1].
  double Ramp(double t) const;

  // Configuration. Default member-inits keep a default-constructed instance
  // benign until SetParameters() runs (the gz-plugin loader path).
  double        tileSize_{200.0};
  std::size_t   gridSize_{128};
  double        windSpeed_{0.0};
  double        windDirX_{1.0};
  double        windDirY_{0.0};
  double        gain_{1.0};
  double        tau_{2.0};

  // Base spectrum amplitudes h0(k), generated once
  Eigen::MatrixXcd h0_;
  Eigen::MatrixXcd h0Conj_;

  // Per-update height grid (real domain), regenerated by Update(t)
  Eigen::MatrixXd heightGrid_;
  // Per-update horizontal displacement grids (Tessendorf eq. 29). Used by
  // the visual shader to produce choppy, asymmetric wave crests.
  Eigen::MatrixXd dispXGrid_;
  Eigen::MatrixXd dispYGrid_;
  // Folding / whitecap metric (Encino path): per-cell minimum eigenvalue of
  // the displacement Jacobian, 1 = flat. Sampled by Jacobian() → FoamMask().
  Eigen::MatrixXd minEGrid_;
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

  // Column-major view into the grids above, returned by Field() as the
  // backend-agnostic rendering contract. Repopulated on each call from the
  // current grid data() — Update may reallocate the grids (the Phillips path
  // reassigns heightGrid_), so a cached pointer would dangle. Mutable because
  // Field() is const.
  mutable WaveField2D field_;

  // Per-axis wavenumber arrays kx[i], ky[j] (precomputed)
  Eigen::VectorXd kxRow_;
  Eigen::VectorXd kyCol_;

  // Per-update dispersion factor omega(k) (precomputed)
  Eigen::MatrixXd omegaGrid_;

  // Optional EncinoWaves-backed Update path. The EncinoState type is
  // defined entirely in FFTWaveSimulation.cc so the vendored EncinoWaves
  // headers don't leak into the public include surface here.
  bool useEncino_{false};
  struct EncinoState;
  std::unique_ptr<EncinoState> encino_;

  // Physics-based amplitude calibration for the Encino path. EncinoWaves'
  // `amplitudeGain` does not scale its height/displacement output, and its
  // intrinsic field variance is far larger than a physical sea state at our
  // wind speeds. We measure Encino's intrinsic RMS once at construction and
  // store the factor that rescales the significant wave height to the
  // standard fully-developed Pierson-Moskowitz wind-sea relation
  // (Hs = 0.21 * V19.5^2 / g). Applied to η/Dx/Dy every Update. 1.0 on the
  // Phillips path (unused).
  double encinoScale_{1.0};

  // When true (default), Update() computes the 5 derivative grids in
  // addition to η/Dx/Dy. Toggled off by consumers that don't read them
  // (e.g. the GPU-FFT visual path that does finite-diff normals).
  bool computeDerivatives_{true};

  // Time of the last Update(). The field at a given time is deterministic, so
  // Update() short-circuits on a repeat call for the same time. This lets
  // several consumers (the Waves system plus each WaveBuoyancy that advances
  // the field instance it holds — see the cross-process note in WaveBuoyancy)
  // converge on one FFT per tick instead of one per consumer. -1 = never
  // updated (sim time is always >= 0).
  double lastUpdateT_{-1.0};
};

/// \brief Factory: a default-constructed FFT wave-field engine (apply
/// `SetParameters` before use). Registered under the "fft" token so
/// `CreateWaveSimulation` can rebuild the engine from a serialized `Wavefield`
/// component on the GUI side.
std::shared_ptr<IWaveField> MakeFFTWaveField();

}  // namespace gz::sim::waves

#endif  // GZ_SIM_WAVES_FFTWAVESIMULATION_HH_
