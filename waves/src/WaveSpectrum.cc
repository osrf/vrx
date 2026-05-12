/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include "gz/sim/waves/WaveSpectrum.hh"

#include <cmath>
#include <iostream>

#include <gz/math/Helpers.hh>

namespace gz::sim::waves
{

namespace
{
constexpr double kGravity = 9.80665;
}

double DeepWaterDispersionToOmega(double wavenumber)
{
  return std::sqrt(kGravity * wavenumber);
}

double DeepWaterDispersionToWavenumber(double omega)
{
  return omega * omega / kGravity;
}

double PiersonMoskowitz(double omega, double omegaP)
{
  // S(ω) = α·g²/ω⁵ · exp(-1.25·(ωP/ω)⁴)
  constexpr double kAlpha = 0.0081;
  const double ratio = omegaP / omega;
  return kAlpha * kGravity * kGravity / std::pow(omega, 5.0) *
         std::exp(-1.25 * std::pow(ratio, 4.0));
}

namespace
{

void SamplePms(WavefieldData &data)
{
  const auto &p = data.params;
  const double omegaMean = 2.0 * M_PI / p.period;
  const std::size_t n = p.number;

  // Frequency-domain integration widths for the three sample points
  // (mean / scale, mean, mean * scale). Equal energy quadrature.
  std::vector<double> dOmega(n, 0.0);
  if (n == 3)
  {
    dOmega[0] = omegaMean * (1.0 - 1.0 / p.scale);
    dOmega[1] = omegaMean * (p.scale - 1.0 / p.scale) * 0.5;
    dOmega[2] = omegaMean * (p.scale - 1.0);
  }
  else
  {
    // Fallback for N != 3: equal spacing around the mean.
    const double width = omegaMean * (p.scale - 1.0 / p.scale) /
                         static_cast<double>(n);
    for (std::size_t i = 0; i < n; ++i)
      dOmega[i] = width;
  }

  data.amplitudes.resize(n);
  data.wavenumbers.resize(n);
  data.angularFrequencies.resize(n);
  data.steepnesses.resize(n);
  data.directions.resize(n);

  for (std::size_t i = 0; i < n; ++i)
  {
    const int nIdx = static_cast<int>(i) - static_cast<int>(n / 2);
    const double scaleFactor = std::pow(p.scale, nIdx);
    const double omega = omegaMean * scaleFactor;
    const double pms = PiersonMoskowitz(omega, omegaMean);
    const double a = p.gain * std::sqrt(2.0 * pms * dOmega[i]);
    const double k = DeepWaterDispersionToWavenumber(omega);
    double q = 0.0;
    if (!gz::math::equal(a, 0.0))
    {
      q = std::min(1.0, p.steepness / (a * k * static_cast<double>(n)));
    }

    data.amplitudes[i] = a;
    data.wavenumbers[i] = k;
    data.angularFrequencies[i] = omega;
    data.steepnesses[i] = q;

    const double theta = nIdx * p.angle + p.direction;
    data.directions[i] = gz::math::Vector2d(std::cos(theta), std::sin(theta));
  }
}

void SampleCwr(WavefieldData &data)
{
  const auto &p = data.params;
  const double omegaMean = 2.0 * M_PI / p.period;
  const double kMean = DeepWaterDispersionToWavenumber(omegaMean);
  const std::size_t n = p.number;

  data.amplitudes.resize(n);
  data.wavenumbers.resize(n);
  data.angularFrequencies.resize(n);
  data.steepnesses.resize(n);
  data.directions.resize(n);

  for (std::size_t i = 0; i < n; ++i)
  {
    const int nIdx = static_cast<int>(i) - static_cast<int>(n / 2);
    const double scaleFactor = std::pow(p.scale, nIdx);
    const double a = scaleFactor * p.amplitude;
    const double k = kMean / scaleFactor;
    const double omega = DeepWaterDispersionToOmega(k);
    double q = 0.0;
    if (!gz::math::equal(a, 0.0))
    {
      q = std::min(1.0, p.steepness / (a * k * static_cast<double>(n)));
    }

    data.amplitudes[i] = a;
    data.wavenumbers[i] = k;
    data.angularFrequencies[i] = omega;
    data.steepnesses[i] = q;

    const double theta = nIdx * p.angle + p.direction;
    data.directions[i] = gz::math::Vector2d(std::cos(theta), std::sin(theta));
  }
}

}  // namespace

void SampleSpectrum(WavefieldData &data)
{
  if (data.params.model == "PMS")
  {
    SamplePms(data);
  }
  else if (data.params.model == "CWR")
  {
    SampleCwr(data);
  }
  else
  {
    std::cerr << "[gz::sim::waves] Unknown spectrum model '"
              << data.params.model << "'. Expected 'PMS' or 'CWR'."
              << std::endl;
    data.amplitudes.clear();
    data.wavenumbers.clear();
    data.angularFrequencies.clear();
    data.steepnesses.clear();
    data.directions.clear();
    return;
  }

  ++data.generation;
}

}  // namespace gz::sim::waves
