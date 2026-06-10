/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include "gz/sim/waves/GerstnerWaveSimulation.hh"

#include <algorithm>
#include <cmath>
#include <iostream>


#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::waves
{

namespace
{

//////////////////////////////////////////////////
double DispersionOmega(double k, double g)
{
  return std::sqrt(g * k);
}

//////////////////////////////////////////////////
double DispersionWavenumber(double omega, double g)
{
  return omega * omega / g;
}

//////////////////////////////////////////////////
double PiersonMoskowitz(double omega, double omegaP, double g)
{
  constexpr double kAlpha = 0.0081;
  const double ratio = omegaP / omega;
  return kAlpha * g * g / std::pow(omega, 5.0) *
         std::exp(-1.25 * std::pow(ratio, 4.0));
}

}  // namespace

GerstnerWaveSimulation::GerstnerWaveSimulation() = default;

//////////////////////////////////////////////////
GerstnerWaveSimulation::GerstnerWaveSimulation(const WaveParameters &p)
{
  this->SetParameters(p);
}

//////////////////////////////////////////////////
void GerstnerWaveSimulation::SetParameters(const WaveParameters &_params)
{
  // Resolve <sea_state> (if set) into period/gain before configuring.
  const WaveParameters p = WithSeaState(_params);
  this->tau_ = p.tau;
  this->phase_ = p.phase;
  const double omegaMean = 2.0 * M_PI / p.period;
  const std::size_t n = p.number;

  amplitudes_.resize(n);
  wavenumbers_.resize(n);
  angularFrequencies_.resize(n);
  steepnesses_.resize(n);
  directions_.resize(n);

  // Pre-compute spectral-bin widths once. The 3-component PMS sampler is the
  // standard layout from asv_wave_sim / Tessendorf; if N != 3 we fall back
  // to uniform spacing around the mean.
  std::vector<double> dOmega(n);
  if (p.model == "PMS")
  {
    if (n == 3)
    {
      dOmega[0] = omegaMean * (1.0 - 1.0 / p.scale);
      dOmega[1] = omegaMean * (p.scale - 1.0 / p.scale) * 0.5;
      dOmega[2] = omegaMean * (p.scale - 1.0);
    }
    else
    {
      const double w =
        omegaMean * (p.scale - 1.0 / p.scale) / static_cast<double>(n);
      std::fill(dOmega.begin(), dOmega.end(), w);
    }
  }

  for (std::size_t i = 0; i < n; ++i)
  {
    const int nIdx = static_cast<int>(i) - static_cast<int>(n / 2);
    const double scaleFactor = std::pow(p.scale, nIdx);

    double a = 0.0;
    double k = 0.0;
    double omega = 0.0;

    if (p.model == "PMS")
    {
      omega = omegaMean * scaleFactor;
      const double pms = PiersonMoskowitz(omega, omegaMean, p.gravity);
      a = p.gain * std::sqrt(2.0 * pms * dOmega[i]);
      k = DispersionWavenumber(omega, p.gravity);
    }
    else if (p.model == "CWR")
    {
      a = scaleFactor * p.amplitude;
      k = DispersionWavenumber(omegaMean, p.gravity) / scaleFactor;
      omega = DispersionOmega(k, p.gravity);
    }
    else
    {
      std::cerr << "[GerstnerWaveSimulation] unknown spectrum model '"
                << p.model << "'; expected 'PMS' or 'CWR'." << '\n';
      amplitudes_.clear();
      wavenumbers_.clear();
      angularFrequencies_.clear();
      steepnesses_.clear();
      directions_.clear();
      return;
    }

    double q = 0.0;
    if (std::abs(a) > 1e-9)
    {
      q = std::min(1.0, p.steepness / (a * k * static_cast<double>(n)));
    }

    amplitudes_[i]         = a;
    wavenumbers_[i]        = k;
    angularFrequencies_[i] = omega;
    steepnesses_[i]        = q;

    const double theta = nIdx * p.angle + p.direction;
    directions_[i] = Eigen::Vector2d(std::cos(theta), std::sin(theta));
  }

  // Render grid: sample the analytic field onto a tile sized to the longest
  // component wavelength so it tiles approximately seamlessly (the shorter
  // components are not exactly periodic over it — minor seams, an accepted
  // trade-off for the secondary analytic backend). Resolution reuses the
  // gridSize knob; the buffers are filled by Update().
  this->fieldN_ = p.gridSize > 0 ? p.gridSize : 128;
  double kMin = 0.0;
  for (const double k : this->wavenumbers_)
    if (k > 0.0 && (kMin == 0.0 || k < kMin)) kMin = k;
  this->fieldTile_ = (kMin > 0.0) ? (2.0 * M_PI / kMin) : 100.0;
  const std::size_t cells = this->fieldN_ * this->fieldN_;
  this->dzBuf_.assign(cells, 0.0);
  this->dxBuf_.assign(cells, 0.0);
  this->dyBuf_.assign(cells, 0.0);
  this->foamBuf_.assign(cells, 0.0);
  this->field_.n    = this->fieldN_;
  this->field_.tile = this->fieldTile_;
  this->field_.dz   = this->dzBuf_.data();
  this->field_.dx   = this->dxBuf_.data();
  this->field_.dy   = this->dyBuf_.data();
  this->field_.foam = this->foamBuf_.data();
  this->currentTime_ = -1.0;  // force the first Update to sample
  this->Update(0.0);
}

//////////////////////////////////////////////////
double GerstnerWaveSimulation::Elevation(double x, double y, double t) const
{
  double eta = 0.0;
  const std::size_t n = amplitudes_.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &d = directions_[i];
    const double theta =
      wavenumbers_[i] * (d.x() * x + d.y() * y) - angularFrequencies_[i] * t +
      phase_;
    eta += amplitudes_[i] * std::cos(theta);
  }
  return eta * StartupRamp(t, this->tau_);
}

//////////////////////////////////////////////////
Eigen::Vector3d GerstnerWaveSimulation::ParticleVelocity(
  double x, double y, double t) const
{
  double vx = 0.0, vy = 0.0, vz = 0.0;
  const double r = StartupRamp(t, this->tau_);
  const std::size_t n = amplitudes_.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &d = directions_[i];
    const double aw = amplitudes_[i] * angularFrequencies_[i];
    const double theta =
      wavenumbers_[i] * (d.x() * x + d.y() * y) - angularFrequencies_[i] * t +
      phase_;
    const double s = std::sin(theta);
    const double c = std::cos(theta);
    vx += aw * d.x() * s;
    vy += aw * d.y() * s;
    vz += aw * c;
  }
  return Eigen::Vector3d(vx * r, vy * r, vz * r);
}

//////////////////////////////////////////////////
Eigen::Vector3d GerstnerWaveSimulation::Normal(
  double x, double y, double t) const
{
  double dhdx = 0.0, dhdy = 0.0;
  const double r = StartupRamp(t, this->tau_);
  const std::size_t n = amplitudes_.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &d = directions_[i];
    const double ak = amplitudes_[i] * wavenumbers_[i];
    const double theta =
      wavenumbers_[i] * (d.x() * x + d.y() * y) - angularFrequencies_[i] * t +
      phase_;
    const double s = std::sin(theta);
    dhdx += -ak * d.x() * s;
    dhdy += -ak * d.y() * s;
  }
  dhdx *= r;
  dhdy *= r;
  Eigen::Vector3d nvec(-dhdx, -dhdy, 1.0);
  nvec.normalize();
  return nvec;
}

//////////////////////////////////////////////////
double GerstnerWaveSimulation::Jacobian(double x, double y, double t) const
{
  double dsxdx = 0.0, dsydy = 0.0, dsxdy = 0.0;
  const double r = StartupRamp(t, this->tau_);
  const std::size_t n = amplitudes_.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &d = directions_[i];
    const double qak = steepnesses_[i] * amplitudes_[i] * wavenumbers_[i];
    const double theta =
      wavenumbers_[i] * (d.x() * x + d.y() * y) - angularFrequencies_[i] * t +
      phase_;
    const double c = std::cos(theta);
    dsxdx += -qak * d.x() * d.x() * c;
    dsydy += -qak * d.y() * d.y() * c;
    dsxdy += -qak * d.x() * d.y() * c;
  }
  dsxdx *= r;
  dsydy *= r;
  dsxdy *= r;
  return (1.0 + dsxdx) * (1.0 + dsydy) - dsxdy * dsxdy;
}

//////////////////////////////////////////////////
void GerstnerWaveSimulation::Update(double t)
{
  // Analytic backend: the point queries (Elevation/Normal/...) stay
  // closed-form. Update only (re)samples the render grid at the new time,
  // matching the "Update advances, Field reads" model the FFT backend uses.
  if (t == this->currentTime_)
    return;
  this->currentTime_ = t;

  const int N = static_cast<int>(this->fieldN_);
  if (N <= 0 || this->dzBuf_.empty())
    return;
  const double T = this->fieldTile_;
  const double r = StartupRamp(t, this->tau_);
  const std::size_t nc = this->amplitudes_.size();
  for (int j = 0; j < N; ++j)
  {
    const double y = static_cast<double>(j) * T / static_cast<double>(N);
    for (int i = 0; i < N; ++i)
    {
      const double x = static_cast<double>(i) * T / static_cast<double>(N);
      double dz = 0.0, dx = 0.0, dy = 0.0;
      double dsxdx = 0.0, dsydy = 0.0, dsxdy = 0.0;
      for (std::size_t c = 0; c < nc; ++c)
      {
        const auto &d = this->directions_[c];
        const double theta =
          this->wavenumbers_[c] * (d.x() * x + d.y() * y) -
          this->angularFrequencies_[c] * t + this->phase_;
        const double s  = std::sin(theta);
        const double co = std::cos(theta);
        dz += this->amplitudes_[c] * co;
        // Gerstner horizontal displacement (chop): -q·a·dir·sin(θ).
        const double qa = this->steepnesses_[c] * this->amplitudes_[c];
        dx -= qa * d.x() * s;
        dy -= qa * d.y() * s;
        // Folding/foam terms reuse co — equivalent to Jacobian(x, y, t) but
        // without a second pass over the components.
        const double qak = qa * this->wavenumbers_[c];
        dsxdx -= qak * d.x() * d.x() * co;
        dsydy -= qak * d.y() * d.y() * co;
        dsxdy -= qak * d.x() * d.y() * co;
      }
      const std::size_t idx = static_cast<std::size_t>(i) +
                              static_cast<std::size_t>(j) *
                              static_cast<std::size_t>(N);
      this->dzBuf_[idx]   = dz * r;
      this->dxBuf_[idx]   = dx * r;
      this->dyBuf_[idx]   = dy * r;
      const double jxx = dsxdx * r, jyy = dsydy * r, jxy = dsxdy * r;
      this->foamBuf_[idx] = (1.0 + jxx) * (1.0 + jyy) - jxy * jxy;
    }
  }
}

//////////////////////////////////////////////////
const WaveField2D *GerstnerWaveSimulation::Field() const
{
  return &this->field_;
}

//////////////////////////////////////////////////
/// \brief Factory used to register the Gerstner engine under the "gerstner"
/// token (see RegisterWaveEngineFactory). Returns a default-constructed engine;
/// the caller applies SetParameters.
std::shared_ptr<IWaveField> MakeGerstnerWaveField()
{
  return std::make_shared<GerstnerWaveSimulation>();
}

}  // namespace gz::sim::waves
