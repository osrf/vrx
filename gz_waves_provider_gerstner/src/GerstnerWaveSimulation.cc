/*
 * Copyright (C) 2026 Honu Robotics
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

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>  // GZ_PI

#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::waves
{

namespace
{

//////////////////////////////////////////////////
double DispersionOmega(double _k, double _g)
{
  return std::sqrt(_g * _k);
}

//////////////////////////////////////////////////
double DispersionWavenumber(double _omega, double _g)
{
  return _omega * _omega / _g;
}

//////////////////////////////////////////////////
double PiersonMoskowitz(double _omega, double _omegaP, double _g)
{
  constexpr double kAlpha = 0.0081;
  const double ratio = _omegaP / _omega;
  return kAlpha * _g * _g / std::pow(_omega, 5.0) *
         std::exp(-1.25 * std::pow(ratio, 4.0));
}

}  // namespace

//////////////////////////////////////////////////
GerstnerWaveSimulation::GerstnerWaveSimulation() = default;

//////////////////////////////////////////////////
GerstnerWaveSimulation::GerstnerWaveSimulation(const WaveParameters &_p)
{
  this->SetParameters(_p);
}

//////////////////////////////////////////////////
void GerstnerWaveSimulation::SetParameters(const WaveParameters &_params)
{
  // Resolve <sea_state> (if set) into period/gain before configuring.
  const WaveParameters p = WithSeaState(_params);

  // A non-positive period makes omegaMean (2*pi/period) infinite/garbage and
  // the Pierson-Moskowitz spectrum (∝ 1/omega^5) blow up. Reject it and leave
  // the field flat rather than emit NaNs into elevation/normals downstream.
  if (!(p.period > 0.0))
  {
    gzerr << "[GerstnerWaveSimulation] non-positive <period> (" << p.period
          << "); leaving the wave field flat." << '\n';
    this->amplitudes.clear();
    this->wavenumbers.clear();
    this->angularFrequencies.clear();
    this->steepnesses.clear();
    this->directions.clear();
    return;
  }

  this->tau = p.tau;
  this->phase = p.phase;
  const double omegaMean = 2.0 * GZ_PI / p.period;
  const std::size_t n = p.number;

  this->amplitudes.resize(n);
  this->wavenumbers.resize(n);
  this->angularFrequencies.resize(n);
  this->steepnesses.resize(n);
  this->directions.resize(n);

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
      gzerr << "[GerstnerWaveSimulation] unknown spectrum model '"
            << p.model << "'; expected 'PMS' or 'CWR'." << '\n';
      this->amplitudes.clear();
      this->wavenumbers.clear();
      this->angularFrequencies.clear();
      this->steepnesses.clear();
      this->directions.clear();
      return;
    }

    double q = 0.0;
    if (std::abs(a) > 1e-9)
    {
      q = std::min(1.0, p.steepness / (a * k * static_cast<double>(n)));
    }

    this->amplitudes[i]         = a;
    this->wavenumbers[i]        = k;
    this->angularFrequencies[i] = omega;
    this->steepnesses[i]        = q;

    const double theta = nIdx * p.angle + p.direction;
    this->directions[i] = gz::math::Vector2d(std::cos(theta), std::sin(theta));
  }

  // Render grid: sample the analytic field onto a tile sized to the longest
  // component wavelength. Resolution reuses the gridSize knob; the buffers are
  // filled by Update().
  this->fieldN = p.gridSize > 0 ? p.gridSize : 128;
  double kMin = 0.0;
  for (const double k : this->wavenumbers)
    if (k > 0.0 && (kMin == 0.0 || k < kMin)) kMin = k;
  this->fieldTile = (kMin > 0.0) ? (2.0 * GZ_PI / kMin) : 100.0;

  // Quantize every component's wave vector to the tile lattice so each one
  // completes a whole number of cycles over this->fieldTile (not just the longest).
  // This makes the analytic field exactly periodic over the tile, so the
  // rendered surface tiles with no edge seams. The fundamental is
  // k0 = 2*pi/this->fieldTile (== kMin); snapping (kx, ky) to the nearest integer
  // multiples of k0 shifts each component by at most half a lattice step.
  // (|k| >= kMin guarantees the snapped vector is never zero.) omega and the
  // steepness clamp are recomputed from the snapped wavenumber.
  if (kMin > 0.0)
  {
    const double k0 = 2.0 * GZ_PI / this->fieldTile;
    for (std::size_t i = 0; i < n; ++i)
    {
      const gz::math::Vector2d kVec =
        this->wavenumbers[i] * this->directions[i];
      const gz::math::Vector2d kSnap(std::round(kVec.X() / k0) * k0,
                                     std::round(kVec.Y() / k0) * k0);
      const double kMag = kSnap.Length();
      if (kMag <= 0.0)
        continue;
      this->wavenumbers[i]        = kMag;
      this->directions[i]         = kSnap / kMag;
      this->angularFrequencies[i] = DispersionOmega(kMag, p.gravity);
      if (std::abs(this->amplitudes[i]) > 1e-9)
        this->steepnesses[i] = std::min(1.0,
          p.steepness / (this->amplitudes[i] * kMag * static_cast<double>(n)));
    }
  }
  const std::size_t cells = this->fieldN * this->fieldN;
  this->dzBuf.assign(cells, 0.0);
  this->dxBuf.assign(cells, 0.0);
  this->dyBuf.assign(cells, 0.0);
  this->foamBuf.assign(cells, 0.0);
  this->field.n    = this->fieldN;
  this->field.tile = this->fieldTile;
  this->field.dz   = this->dzBuf.data();
  this->field.dx   = this->dxBuf.data();
  this->field.dy   = this->dyBuf.data();
  this->field.foam = this->foamBuf.data();
  this->currentTime = -1.0;  // force the first Update to sample
  this->Update(0.0);
}

//////////////////////////////////////////////////
double GerstnerWaveSimulation::Phase(
  std::size_t _i, double _x, double _y, double _t) const
{
  const auto &d = this->directions[_i];
  return this->wavenumbers[_i] * (d.X() * _x + d.Y() * _y) -
         this->angularFrequencies[_i] * _t + this->phase;
}

//////////////////////////////////////////////////
double GerstnerWaveSimulation::Elevation(double _x, double _y, double _t) const
{
  double eta = 0.0;
  const std::size_t n = this->amplitudes.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    eta += this->amplitudes[i] * std::cos(this->Phase(i, _x, _y, _t));
  }
  return eta * StartupRamp(_t, this->tau);
}

//////////////////////////////////////////////////
gz::math::Vector3d GerstnerWaveSimulation::ParticleVelocity(
  double _x, double _y, double _t) const
{
  double vx = 0.0, vy = 0.0, vz = 0.0;
  const double r = StartupRamp(_t, this->tau);
  const std::size_t n = this->amplitudes.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &d = this->directions[i];
    const double aw = this->amplitudes[i] * this->angularFrequencies[i];
    const double theta = this->Phase(i, _x, _y, _t);
    const double s = std::sin(theta);
    const double c = std::cos(theta);
    vx += aw * d.X() * s;
    vy += aw * d.Y() * s;
    vz += aw * c;
  }
  return gz::math::Vector3d(vx * r, vy * r, vz * r);
}

//////////////////////////////////////////////////
gz::math::Vector3d GerstnerWaveSimulation::Normal(
  double _x, double _y, double _t) const
{
  double dhdx = 0.0, dhdy = 0.0;
  const double r = StartupRamp(_t, this->tau);
  const std::size_t n = this->amplitudes.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &d = this->directions[i];
    const double ak = this->amplitudes[i] * this->wavenumbers[i];
    const double theta = this->Phase(i, _x, _y, _t);
    const double s = std::sin(theta);
    dhdx += -ak * d.X() * s;
    dhdy += -ak * d.Y() * s;
  }
  dhdx *= r;
  dhdy *= r;
  gz::math::Vector3d nvec(-dhdx, -dhdy, 1.0);
  nvec.Normalize();
  return nvec;
}

//////////////////////////////////////////////////
double GerstnerWaveSimulation::Jacobian(double _x, double _y, double _t) const
{
  double dsxdx = 0.0, dsydy = 0.0, dsxdy = 0.0;
  const double r = StartupRamp(_t, this->tau);
  const std::size_t n = this->amplitudes.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &d = this->directions[i];
    const double qak =
      this->steepnesses[i] * this->amplitudes[i] * this->wavenumbers[i];
    const double theta = this->Phase(i, _x, _y, _t);
    const double c = std::cos(theta);
    dsxdx += -qak * d.X() * d.X() * c;
    dsydy += -qak * d.Y() * d.Y() * c;
    dsxdy += -qak * d.X() * d.Y() * c;
  }
  dsxdx *= r;
  dsydy *= r;
  dsxdy *= r;
  return (1.0 + dsxdx) * (1.0 + dsydy) - dsxdy * dsxdy;
}

//////////////////////////////////////////////////
void GerstnerWaveSimulation::Update(double _simTime)
{
  // Analytic engine: the point queries (Elevation/Normal/...) stay
  // closed-form. Update only (re)samples the render grid at the new time,
  // matching the "Update advances, Field reads" model the FFT engine uses.
  if (_simTime == this->currentTime)
    return;
  this->currentTime = _simTime;

  const int N = static_cast<int>(this->fieldN);
  if (N <= 0 || this->dzBuf.empty())
    return;
  const double T = this->fieldTile;
  const double r = StartupRamp(_simTime, this->tau);
  const std::size_t nc = this->amplitudes.size();
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
        const auto &d = this->directions[c];
        const double theta = this->Phase(c, x, y, _simTime);
        const double s  = std::sin(theta);
        const double co = std::cos(theta);
        dz += this->amplitudes[c] * co;
        // Gerstner horizontal displacement (chop): -q·a·dir·sin(θ).
        const double qa = this->steepnesses[c] * this->amplitudes[c];
        dx -= qa * d.X() * s;
        dy -= qa * d.Y() * s;
        // Folding/foam terms reuse co — equivalent to Jacobian(x, y, t) but
        // without a second pass over the components.
        const double qak = qa * this->wavenumbers[c];
        dsxdx -= qak * d.X() * d.X() * co;
        dsydy -= qak * d.Y() * d.Y() * co;
        dsxdy -= qak * d.X() * d.Y() * co;
      }
      const std::size_t idx = static_cast<std::size_t>(i) +
                              static_cast<std::size_t>(j) *
                              static_cast<std::size_t>(N);
      this->dzBuf[idx]   = dz * r;
      this->dxBuf[idx]   = dx * r;
      this->dyBuf[idx]   = dy * r;
      const double jxx = dsxdx * r, jyy = dsydy * r, jxy = dsxdy * r;
      this->foamBuf[idx] = (1.0 + jxx) * (1.0 + jyy) - jxy * jxy;
    }
  }
}

//////////////////////////////////////////////////
const WaveField2D *GerstnerWaveSimulation::Field() const
{
  return &this->field;
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
