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

#include <gz/math/Helpers.hh>

#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::waves
{

namespace
{
constexpr double kGravity = 9.80665;

double DispersionOmega(double k)
{
  return std::sqrt(kGravity * k);
}

double DispersionWavenumber(double omega)
{
  return omega * omega / kGravity;
}

double PiersonMoskowitz(double omega, double omegaP)
{
  constexpr double kAlpha = 0.0081;
  const double ratio = omegaP / omega;
  return kAlpha * kGravity * kGravity / std::pow(omega, 5.0) *
         std::exp(-1.25 * std::pow(ratio, 4.0));
}

}  // namespace

GerstnerWaveSimulation::GerstnerWaveSimulation(const WaveParameters &p)
  : tau_(p.tau)
{
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
      const double pms = PiersonMoskowitz(omega, omegaMean);
      a = p.gain * std::sqrt(2.0 * pms * dOmega[i]);
      k = DispersionWavenumber(omega);
    }
    else if (p.model == "CWR")
    {
      a = scaleFactor * p.amplitude;
      k = DispersionWavenumber(omegaMean) / scaleFactor;
      omega = DispersionOmega(k);
    }
    else
    {
      std::cerr << "[GerstnerWaveSimulation] unknown spectrum model '"
                << p.model << "'; expected 'PMS' or 'CWR'." << std::endl;
      amplitudes_.clear();
      wavenumbers_.clear();
      angularFrequencies_.clear();
      steepnesses_.clear();
      directions_.clear();
      return;
    }

    double q = 0.0;
    if (!gz::math::equal(a, 0.0))
    {
      q = std::min(1.0, p.steepness / (a * k * static_cast<double>(n)));
    }

    amplitudes_[i]         = a;
    wavenumbers_[i]        = k;
    angularFrequencies_[i] = omega;
    steepnesses_[i]        = q;

    const double theta = nIdx * p.angle + p.direction;
    directions_[i] = gz::math::Vector2d(std::cos(theta), std::sin(theta));
  }
}

double GerstnerWaveSimulation::Ramp(double t) const
{
  if (tau_ <= 0.0)
    return 1.0;
  return 1.0 - std::exp(-t / tau_);
}

double GerstnerWaveSimulation::Elevation(double x, double y, double t) const
{
  double eta = 0.0;
  const std::size_t n = amplitudes_.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &d = directions_[i];
    const double theta =
      wavenumbers_[i] * (d.X() * x + d.Y() * y) - angularFrequencies_[i] * t;
    eta += amplitudes_[i] * std::cos(theta);
  }
  return eta * Ramp(t);
}

gz::math::Vector3d GerstnerWaveSimulation::ParticleVelocity(
  double x, double y, double t) const
{
  double vx = 0.0, vy = 0.0, vz = 0.0;
  const double r = Ramp(t);
  const std::size_t n = amplitudes_.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &d = directions_[i];
    const double aw = amplitudes_[i] * angularFrequencies_[i];
    const double theta =
      wavenumbers_[i] * (d.X() * x + d.Y() * y) - angularFrequencies_[i] * t;
    const double s = std::sin(theta);
    const double c = std::cos(theta);
    vx += aw * d.X() * s;
    vy += aw * d.Y() * s;
    vz += aw * c;
  }
  return {vx * r, vy * r, vz * r};
}

gz::math::Vector3d GerstnerWaveSimulation::Normal(
  double x, double y, double t) const
{
  double dhdx = 0.0, dhdy = 0.0;
  const double r = Ramp(t);
  const std::size_t n = amplitudes_.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &d = directions_[i];
    const double ak = amplitudes_[i] * wavenumbers_[i];
    const double theta =
      wavenumbers_[i] * (d.X() * x + d.Y() * y) - angularFrequencies_[i] * t;
    const double s = std::sin(theta);
    dhdx += -ak * d.X() * s;
    dhdy += -ak * d.Y() * s;
  }
  dhdx *= r;
  dhdy *= r;
  gz::math::Vector3d nvec{-dhdx, -dhdy, 1.0};
  nvec.Normalize();
  return nvec;
}

double GerstnerWaveSimulation::Jacobian(double x, double y, double t) const
{
  double dsxdx = 0.0, dsydy = 0.0, dsxdy = 0.0;
  const double r = Ramp(t);
  const std::size_t n = amplitudes_.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &d = directions_[i];
    const double qak = steepnesses_[i] * amplitudes_[i] * wavenumbers_[i];
    const double theta =
      wavenumbers_[i] * (d.X() * x + d.Y() * y) - angularFrequencies_[i] * t;
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

}  // namespace gz::sim::waves
