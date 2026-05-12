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
#include <random>

#include <unsupported/Eigen/FFT>

#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::waves
{

namespace
{
constexpr double kGravity = 9.80665;
constexpr double k2Pi    = 6.28318530717958647692;
}  // namespace

FFTWaveSimulation::FFTWaveSimulation(const WaveParameters &p,
                                     double tile,
                                     std::size_t grid,
                                     std::uint32_t seed)
  : tileSize_(tile)
  , gridSize_(grid)
  , gain_(p.gain)
  , tau_(p.tau)
{
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

  this->heightGrid_ = Eigen::MatrixXd::Zero(N, N);
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

void FFTWaveSimulation::Update(double t)
{
  const int N = static_cast<int>(this->gridSize_);
  const double ramp = this->Ramp(t);

  // Evolve spectrum: h(k, t) = h0(k)·exp(i·ω·t) + h0_conj(k)·exp(-i·ω·t).
  Eigen::MatrixXcd hkt(N, N);
  for (int i = 0; i < N; ++i)
  {
    for (int j = 0; j < N; ++j)
    {
      const double w = this->omegaGrid_(i, j);
      const std::complex<double> e_plus(std::cos(w * t), std::sin(w * t));
      const std::complex<double> e_minus = std::conj(e_plus);
      hkt(i, j) = this->h0_(i, j) * e_plus +
                  this->h0Conj_(i, j) * e_minus;
    }
  }

  // 2D inverse FFT via row-then-column passes (Eigen::FFT is 1D).
  Eigen::FFT<double> fft;
  Eigen::MatrixXcd rowOut(N, N);
  for (int i = 0; i < N; ++i)
  {
    Eigen::VectorXcd freq = hkt.row(i);
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

  // Real part is the height field. Apply the startup ramp.
  this->heightGrid_ = colOut.real() * ramp;
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
