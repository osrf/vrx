/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include "gz/sim/waves/FFTWaveSimulation.hh"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>

#include <unsupported/Eigen/FFT>

#include <oneapi/tbb/global_control.h>

#include "gz/sim/waves/Wavefield.hh"

#include "EncinoWaves/All.h"

namespace gz::sim::waves
{

namespace
{
constexpr double kGravity = 9.80665;
constexpr double k2Pi    = 6.28318530717958647692;

/// \brief Process-wide cap on the TBB worker pool.
///
/// Encino's `propagate()` farms its per-cell spectrum work out via
/// `tbb::parallel_for`. At 30 Hz with N=128 there's only ~5–10 ms of
/// useful parallel work per call — far less than enough to keep the
/// default pool (1× `hardware_concurrency()`) usefully busy. On a
/// 24-core machine the extra 20+ workers idle-wait on
/// `wait_bounded_queue_monitor`, which `perf` reports as ~60% of
/// total server CPU. That wasted wall time directly limits RTF.
///
/// Cap the pool at 4 threads, which is enough to absorb Encino's
/// inner parallelism without leaving an army of idle spinners.
/// `tbb::global_control` stacks across instances with `min` semantics,
/// so leaking this singleton just locks in the cap. (Process-wide is
/// fine — other consumers in the simulation that use TBB will benefit
/// from the same cap.)
void EnsureTbbPoolCapped()
{
  static const tbb::global_control kPool(
      tbb::global_control::max_allowed_parallelism, 4u);
  (void)kPool;
}

/// Returns the integer log2 of n if n is a positive power of two; -1 otherwise.
int Log2Pow2(std::size_t n)
{
  if (n == 0 || (n & (n - 1)) != 0) return -1;
  int r = 0;
  while ((static_cast<std::size_t>(1u) << r) < n) ++r;
  return r;
}

/// True when GZ_WAVES_USE_ENCINO=1 is set at construction time. Read once
/// per FFTWaveSimulation instance — toggling the env var mid-run won't
/// take effect until the wave field gets rebuilt.
bool EncinoEnabledByEnv()
{
  const char *v = std::getenv("GZ_WAVES_USE_ENCINO");
  return v && std::string(v) == "1";
}
}  // namespace

//-----------------------------------------------------------------------------
// EncinoState — pimpl that holds the vendored Horvath-spectrum library's
// per-instance state. Defined here (not in the header) so EncinoWaves headers
// stay out of the public include surface.
//-----------------------------------------------------------------------------
struct FFTWaveSimulation::EncinoState
{
  EncinoWaves::Parametersf params;
  std::unique_ptr<EncinoWaves::InitialStatef> initial;
  std::unique_ptr<EncinoWaves::Propagationf>   propagation;
  std::unique_ptr<EncinoWaves::PropagatedStatef> state;
};

FFTWaveSimulation::~FFTWaveSimulation() = default;

FFTWaveSimulation::FFTWaveSimulation(const WaveParameters &p,
                                     double tile,
                                     std::size_t grid,
                                     std::uint32_t seed)
  : tileSize_(tile)
  , gridSize_(grid)
  , gain_(p.gain)
  , tau_(p.tau)
{
  // Cap TBB's worker pool before Encino starts spinning workers up.
  // Idle workers parked on wait_bounded_queue_monitor were ~60% of
  // server CPU before this.
  EnsureTbbPoolCapped();

  // PMS relation: peak omega <-> wind speed at 19.5 m. We have period →
  // omegaP → V19. (deep-water dispersion places the spectral peak at
  // omegaP ≈ 0.879 * g / V19.)
  const double omegaP = k2Pi / p.period;
  this->windSpeed_ = 0.879 * kGravity / omegaP;
  this->windDirX_ = std::cos(p.direction);
  this->windDirY_ = std::sin(p.direction);

  const int N = static_cast<int>(this->gridSize_);
  const double L = this->tileSize_;

  // Wavenumber grid in standard FFT layout: k=0 at index 0, positive k for
  // i < N/2, negative k for i ≥ N/2. (Not fftshift-centered — Eigen::FFT
  // expects this layout natively.)
  this->kxRow_.resize(N);
  this->kyCol_.resize(N);
  for (int i = 0; i < N; ++i)
  {
    const double mi = (i < N / 2) ? static_cast<double>(i)
                                  : static_cast<double>(i - N);
    this->kxRow_[i] = k2Pi * mi / L;
    this->kyCol_[i] = k2Pi * mi / L;
  }

  // omega(k) = √(g·|k|) cached per cell.
  this->omegaGrid_.resize(N, N);
  for (int i = 0; i < N; ++i)
  {
    for (int j = 0; j < N; ++j)
    {
      const double kmag =
        std::hypot(this->kxRow_[i], this->kyCol_[j]);
      this->omegaGrid_(i, j) = std::sqrt(kGravity * kmag);
    }
  }

  // Generate h0(k) from the Phillips spectrum. Tessendorf eq. 24:
  //   h0(k) = (xi_r + i·xi_i) · sqrt(P_h(k) / 2)
  // with xi_r, xi_i ~ N(0, 1). The conjugate copy h0_conj(-k) is stored so
  // Update() can evolve the spectrum without re-indexing per cell.
  std::mt19937 rng(seed);
  std::normal_distribution<double> gauss(0.0, 1.0);

  this->h0_.resize(N, N);
  this->h0Conj_.resize(N, N);
  for (int i = 0; i < N; ++i)
  {
    for (int j = 0; j < N; ++j)
    {
      const double Ph = this->Phillips(this->kxRow_[i], this->kyCol_[j]);
      const double sigma = std::sqrt(std::max(0.0, Ph) * 0.5);
      const std::complex<double> xi(gauss(rng), gauss(rng));
      this->h0_(i, j) = xi * sigma;
    }
  }
  // h0_conj(k) = conj(h0(-k))
  for (int i = 0; i < N; ++i)
  {
    for (int j = 0; j < N; ++j)
    {
      const int ic = (N - i) % N;
      const int jc = (N - j) % N;
      this->h0Conj_(i, j) = std::conj(this->h0_(ic, jc));
    }
  }

  // Continuous-spectrum-to-discrete-IFFT amplitude correction.
  // Variance argument: we want σ²_η = ∫ P(k) dk_x dk_y. The discrete sum
  // gives Σ P(k_n) · (2π/L)², and Eigen's inverse FFT additionally
  // normalises by 1/N². Pre-scaling h0 by `2π·N²/L` makes the per-cell
  // variance recover the continuous integral; the √2 compensates for the
  // (1/√2) factor already baked into the random h0 amplitudes above.
  // Without this fix the resulting η is ~200× too small for typical
  // tile/grid choices (e.g. L=200 m, N=128).
  const double specScale =
      std::sqrt(2.0) * k2Pi * static_cast<double>(N) *
      static_cast<double>(N) / L;
  this->h0_     *= specScale;
  this->h0Conj_ *= specScale;

  this->heightGrid_ = Eigen::MatrixXd::Zero(N, N);
  this->dispXGrid_  = Eigen::MatrixXd::Zero(N, N);
  this->dispYGrid_  = Eigen::MatrixXd::Zero(N, N);
  this->slopeXGrid_   = Eigen::MatrixXd::Zero(N, N);
  this->slopeYGrid_   = Eigen::MatrixXd::Zero(N, N);
  this->dispDxDxGrid_ = Eigen::MatrixXd::Zero(N, N);
  this->dispDyDyGrid_ = Eigen::MatrixXd::Zero(N, N);
  this->dispDxDyGrid_ = Eigen::MatrixXd::Zero(N, N);

  // Optional: bring up the Apache-2.0 EncinoWaves spectrum library. The
  // toggle is intentionally an env var rather than an SDF parameter for
  // now — it's an experiment knob, not a stable interface. The Encino
  // path leaves slope/chop-derivative grids zeroed; consumers must check
  // UseEncino() and skip their slope/chop-deriv upload.
  this->useEncino_ = EncinoEnabledByEnv();
  if (this->useEncino_)
  {
    const int log2N = Log2Pow2(this->gridSize_);
    if (log2N < 0)
    {
      std::cerr << "[FFTWaveSimulation] GZ_WAVES_USE_ENCINO=1 requested but "
                << "gridSize=" << this->gridSize_ << " is not a power of two; "
                << "falling back to Phillips path." << std::endl;
      this->useEncino_ = false;
    }
    else
    {
      this->encino_ = std::make_unique<EncinoState>();
      // Match upstream Horvath defaults except for the four knobs we
      // share with the in-tree Phillips path. EncinoWaves's defaults pick
      // TMA spectrum + Hasselmann directional spread + Capillary
      // dispersion + Normal random distribution, which is the "good
      // ocean" config the Horvath 2015 paper validates against.
      auto &ep = this->encino_->params;
      ep.resolutionPowerOfTwo = log2N;
      ep.domain        = static_cast<float>(this->tileSize_);
      ep.windSpeed     = static_cast<float>(this->windSpeed_);
      ep.amplitudeGain = static_cast<float>(this->gain_);
      ep.random.seed   = static_cast<int>(seed);

      this->encino_->initial =
          std::make_unique<EncinoWaves::InitialStatef>(ep);
      this->encino_->propagation =
          std::make_unique<EncinoWaves::Propagationf>(ep, /*nthreads=*/-1);
      this->encino_->state =
          std::make_unique<EncinoWaves::PropagatedStatef>(ep);

      std::cout << "[FFTWaveSimulation] EncinoWaves spectrum library active "
                << "(res=" << ep.resolution() << " domain=" << ep.domain
                << "m wind=" << ep.windSpeed << "m/s seed=" << ep.random.seed
                << ")" << std::endl;
    }
  }

  this->Update(0.0);
}

double FFTWaveSimulation::Phillips(double kx, double ky) const
{
  const double k2 = kx * kx + ky * ky;
  if (k2 < 1e-12)
    return 0.0;

  const double kmag = std::sqrt(k2);
  const double V = this->windSpeed_;
  const double L = V * V / kGravity;            // wind-fetch length scale
  const double kL2 = (kmag * L) * (kmag * L);

  // Damping at short wavelengths (Tessendorf 2001 §4.3).
  const double l = L * 1e-3;
  const double damp = std::exp(-(k2) * (l * l));

  // Directional weight |k̂ · ŵ|²
  const double khatDotW = (kx * this->windDirX_ + ky * this->windDirY_) / kmag;
  const double directional = khatDotW * khatDotW;

  constexpr double kA = 0.0081;
  return this->gain_ * kA *
         std::exp(-1.0 / std::max(kL2, 1e-12)) /
         (k2 * k2) * directional * damp;
}

double FFTWaveSimulation::Ramp(double t) const
{
  if (this->tau_ <= 0.0)
    return 1.0;
  return 1.0 - std::exp(-t / this->tau_);
}

namespace
{
  // 2D inverse FFT via row-then-column passes (Eigen::FFT is 1D). Returns
  // the real part of the result, multiplied by `scale`.
  Eigen::MatrixXd Ifft2DReal(const Eigen::MatrixXcd &spectrum, double scale)
  {
    const int N = static_cast<int>(spectrum.rows());
    Eigen::FFT<double> fft;
    Eigen::MatrixXcd rowOut(N, N);
    for (int i = 0; i < N; ++i)
    {
      Eigen::VectorXcd freq = spectrum.row(i);
      Eigen::VectorXcd time(N);
      fft.inv(time, freq);
      rowOut.row(i) = time;
    }
    Eigen::MatrixXcd colOut(N, N);
    for (int j = 0; j < N; ++j)
    {
      Eigen::VectorXcd freq = rowOut.col(j);
      Eigen::VectorXcd time(N);
      fft.inv(time, freq);
      colOut.col(j) = time;
    }
    return colOut.real() * scale;
  }
}

void FFTWaveSimulation::Update(double t)
{
  const int N = static_cast<int>(this->gridSize_);

  // ENCINO-BACKED PATH.
  // Drive the heightGrid_/dispXGrid_/dispYGrid_ outputs through the
  // vendored Horvath spectrum library instead of our Phillips path.
  // Slope and chop-derivative grids stay zeroed — the visual layer
  // checks UseEncino() and falls back to finite-diff normals.
  if (this->useEncino_ && this->encino_ && this->encino_->propagation)
  {
    this->encino_->propagation->propagate(
        this->encino_->params,
        *this->encino_->initial,
        *this->encino_->state,
        static_cast<float>(t));

    const float *h  = this->encino_->state->Height.cdata();
    const float *dx = this->encino_->state->Dx.cdata();
    const float *dy = this->encino_->state->Dy.cdata();

    for (int i = 0; i < N; ++i)
    {
      for (int j = 0; j < N; ++j)
      {
        const std::size_t idx =
            static_cast<std::size_t>(i) * N + j;
        this->heightGrid_(i, j) = static_cast<double>(h[idx]);
        this->dispXGrid_(i, j)  = static_cast<double>(dx[idx]);
        this->dispYGrid_(i, j)  = static_cast<double>(dy[idx]);
      }
    }
    return;
  }

  const double ramp = this->Ramp(t);

  // Evolve the height spectrum:
  //   h(k, t) = h0(k)·exp(i·ω·t) + h0_conj(k)·exp(-i·ω·t).
  // Then derive the horizontal-displacement spectra by Tessendorf eq. 29:
  //   Dx(k, t) = -i · (kx / |k|) · h(k, t)
  //   Dy(k, t) = -i · (ky / |k|) · h(k, t)
  // (the unit-vector k̂ projects the displacement into the wave's
  // propagation direction). All three IFFTs share a single ifft2 pass.
  Eigen::MatrixXcd hkt(N, N);
  Eigen::MatrixXcd dxkt(N, N);
  Eigen::MatrixXcd dykt(N, N);
  // Slope / chop-derivative spectra are only filled when a downstream
  // consumer asks for them (visual VS with useSlopeMap=1). Five IFFTs
  // worth of work that's wasted on the GPU-FFT visual path and on
  // anything that finite-differences for normals.
  const bool derivs = this->computeDerivatives_;
  Eigen::MatrixXcd sxkt, sykt, dxdxkt, dydykt, dxdykt;
  if (derivs)
  {
    sxkt.resize(N, N);
    sykt.resize(N, N);
    dxdxkt.resize(N, N);
    dydykt.resize(N, N);
    dxdykt.resize(N, N);
  }
  const std::complex<double> kMinusI(0.0, -1.0);
  const std::complex<double> kPlusI(0.0, 1.0);
  for (int i = 0; i < N; ++i)
  {
    for (int j = 0; j < N; ++j)
    {
      const double w = this->omegaGrid_(i, j);
      const std::complex<double> e_plus(std::cos(w * t), std::sin(w * t));
      const std::complex<double> e_minus = std::conj(e_plus);
      const std::complex<double> h = this->h0_(i, j) * e_plus +
                                     this->h0Conj_(i, j) * e_minus;
      hkt(i, j) = h;

      const double kx = this->kxRow_[i];
      const double ky = this->kyCol_[j];
      const double kmag = std::hypot(kx, ky);
      if (kmag < 1e-12)
      {
        dxkt(i, j) = 0.0;
        dykt(i, j) = 0.0;
        if (derivs)
        {
          dxdxkt(i, j) = 0.0;
          dydykt(i, j) = 0.0;
          dxdykt(i, j) = 0.0;
        }
      }
      else
      {
        const std::complex<double> factor = kMinusI / kmag;
        dxkt(i, j) = factor * kx * h;
        dykt(i, j) = factor * ky * h;
        if (derivs)
        {
          dxdxkt(i, j) = (kx * kx / kmag) * h;
          dydykt(i, j) = (ky * ky / kmag) * h;
          dxdykt(i, j) = (kx * ky / kmag) * h;
        }
      }
      if (derivs)
      {
        sxkt(i, j) = kPlusI * kx * h;
        sykt(i, j) = kPlusI * ky * h;
      }
    }
  }

  this->heightGrid_   = Ifft2DReal(hkt, ramp);
  this->dispXGrid_    = Ifft2DReal(dxkt, ramp);
  this->dispYGrid_    = Ifft2DReal(dykt, ramp);
  if (derivs)
  {
    this->slopeXGrid_   = Ifft2DReal(sxkt, ramp);
    this->slopeYGrid_   = Ifft2DReal(sykt, ramp);
    this->dispDxDxGrid_ = Ifft2DReal(dxdxkt, ramp);
    this->dispDyDyGrid_ = Ifft2DReal(dydykt, ramp);
    this->dispDxDyGrid_ = Ifft2DReal(dxdykt, ramp);
  }
}

double FFTWaveSimulation::BilinearSample(const Eigen::MatrixXd &grid,
                                         double x, double y) const
{
  const int N = static_cast<int>(this->gridSize_);
  const double L = this->tileSize_;

  // Wrap query into [0, L). Grid cell (i, j) corresponds to world position
  // (i·L/N, j·L/N) — the IFFT output's natural layout, no centring shift.
  auto wrap = [L](double v) {
    double r = std::fmod(v, L);
    if (r < 0.0) r += L;
    return r;
  };
  const double wx = wrap(x);
  const double wy = wrap(y);

  // Grid index in [0, N).
  const double fi = wx / L * N;
  const double fj = wy / L * N;
  int i0 = static_cast<int>(std::floor(fi)) % N;
  int j0 = static_cast<int>(std::floor(fj)) % N;
  if (i0 < 0) i0 += N;
  if (j0 < 0) j0 += N;
  const int i1 = (i0 + 1) % N;
  const int j1 = (j0 + 1) % N;
  const double fx = fi - std::floor(fi);
  const double fy = fj - std::floor(fj);

  return grid(i0, j0) * (1 - fx) * (1 - fy)
       + grid(i1, j0) * fx       * (1 - fy)
       + grid(i0, j1) * (1 - fx) * fy
       + grid(i1, j1) * fx       * fy;
}

double FFTWaveSimulation::Elevation(double x, double y, double /*t*/) const
{
  // Caller is expected to have called Update(t) ≤ this tick (the Waves
  // system does this in PreUpdate). The bilinear sample is time-free.
  return this->BilinearSample(this->heightGrid_, x, y);
}

gz::math::Vector3d FFTWaveSimulation::ParticleVelocity(
  double /*x*/, double /*y*/, double /*t*/) const
{
  // Stage 2: stub. Stage 4+ will compute via additional FFTs of i·k·h(k,t)
  // for the horizontal components and ∂η/∂t for the vertical. Returning
  // zero here means drag against still water — not physically accurate yet.
  return gz::math::Vector3d::Zero;
}

gz::math::Vector3d FFTWaveSimulation::Normal(
  double x, double y, double /*t*/) const
{
  // Finite-difference normal from the height grid. Good enough for unit
  // testing; Stage 4 will produce derivative grids via FFT.
  const double L = this->tileSize_;
  const double h = L / static_cast<double>(this->gridSize_);
  const double dx =
    (this->BilinearSample(this->heightGrid_, x + h, y) -
     this->BilinearSample(this->heightGrid_, x - h, y)) / (2.0 * h);
  const double dy =
    (this->BilinearSample(this->heightGrid_, x, y + h) -
     this->BilinearSample(this->heightGrid_, x, y - h)) / (2.0 * h);
  gz::math::Vector3d n{-dx, -dy, 1.0};
  n.Normalize();
  return n;
}

double FFTWaveSimulation::Jacobian(double /*x*/, double /*y*/, double /*t*/) const
{
  // Linear (Airy-style) FFT model has no horizontal displacement → Jacobian
  // is identically 1. Foam mask therefore returns 0 by default.
  return 1.0;
}

}  // namespace gz::sim::waves
