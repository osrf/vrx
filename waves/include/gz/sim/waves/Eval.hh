/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef GZ_SIM_WAVES_EVAL_HH_
#define GZ_SIM_WAVES_EVAL_HH_

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <gz/math/Vector3.hh>

#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::waves
{

/// \brief Startup ramp factor `(1 - exp(-t/tau))`, clamped to [0, 1].
inline double Ramp(const WavefieldData &wf, double t)
{
  if (wf.params.tau <= 0.0)
    return 1.0;
  return 1.0 - std::exp(-t / wf.params.tau);
}

/// \brief Surface elevation η(x, y, t) [m] above the still water level.
/// Linear (Airy) sum of component waves. Exact for steepness = 0.
inline double SurfaceElevation(const WavefieldData &wf,
                               double x, double y, double t)
{
  double eta = 0.0;
  const std::size_t n = wf.amplitudes.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &d = wf.directions[i];
    const double theta =
      wf.wavenumbers[i] * (d.X() * x + d.Y() * y) -
      wf.angularFrequencies[i] * t;
    eta += wf.amplitudes[i] * std::cos(theta);
  }
  return eta * Ramp(wf, t);
}

/// \brief Analytic orbital water velocity at the surface point (x, y) [m/s].
/// Derivative of the Gerstner displacement field. Used by drag terms.
inline gz::math::Vector3d ParticleVelocity(const WavefieldData &wf,
                                           double x, double y, double t)
{
  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
  const double ramp = Ramp(wf, t);
  const std::size_t n = wf.amplitudes.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &d = wf.directions[i];
    const double aw = wf.amplitudes[i] * wf.angularFrequencies[i];
    const double theta =
      wf.wavenumbers[i] * (d.X() * x + d.Y() * y) -
      wf.angularFrequencies[i] * t;
    const double s = std::sin(theta);
    const double c = std::cos(theta);
    vx += aw * d.X() * s;
    vy += aw * d.Y() * s;
    vz += aw * c;
  }
  return {vx * ramp, vy * ramp, vz * ramp};
}

/// \brief Outward-pointing unit normal to the surface at (x, y, t).
/// Analytic derivative of the linear (Airy) surface elevation.
inline gz::math::Vector3d Normal(const WavefieldData &wf,
                                 double x, double y, double t)
{
  double dhdx = 0.0;
  double dhdy = 0.0;
  const double ramp = Ramp(wf, t);
  const std::size_t n = wf.amplitudes.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &d = wf.directions[i];
    const double ak = wf.amplitudes[i] * wf.wavenumbers[i];
    const double theta =
      wf.wavenumbers[i] * (d.X() * x + d.Y() * y) -
      wf.angularFrequencies[i] * t;
    const double s = std::sin(theta);
    dhdx += -ak * d.X() * s;
    dhdy += -ak * d.Y() * s;
  }
  dhdx *= ramp;
  dhdy *= ramp;
  gz::math::Vector3d n_vec{-dhdx, -dhdy, 1.0};
  n_vec.Normalize();
  return n_vec;
}

/// \brief Jacobian determinant of the horizontal displacement field.
/// J < ~0.6 indicates wave folding / whitecap formation (Tessendorf, 2004).
/// For Airy waves (steepness = 0) returns 1; deviates as steepness rises.
inline double Jacobian(const WavefieldData &wf,
                       double x, double y, double t)
{
  double dsxdx = 0.0;
  double dsydy = 0.0;
  double dsxdy = 0.0;
  const double ramp = Ramp(wf, t);
  const std::size_t n = wf.amplitudes.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &d = wf.directions[i];
    const double q = wf.steepnesses[i];
    const double qak = q * wf.amplitudes[i] * wf.wavenumbers[i];
    const double theta =
      wf.wavenumbers[i] * (d.X() * x + d.Y() * y) -
      wf.angularFrequencies[i] * t;
    const double c = std::cos(theta);
    // ∂(s_x)/∂x, ∂(s_y)/∂y, ∂(s_x)/∂y for Gerstner displacement.
    dsxdx += -qak * d.X() * d.X() * c;
    dsydy += -qak * d.Y() * d.Y() * c;
    dsxdy += -qak * d.X() * d.Y() * c;
  }
  dsxdx *= ramp;
  dsydy *= ramp;
  dsxdy *= ramp;
  return (1.0 + dsxdx) * (1.0 + dsydy) - dsxdy * dsxdy;
}

/// \brief Foam intensity in [0, 1] derived from the Jacobian.
/// Above `threshold` returns 0; below returns linearly ramped intensity.
inline double FoamMask(const WavefieldData &wf,
                       double x, double y, double t,
                       double threshold = 0.6)
{
  const double j = Jacobian(wf, x, y, t);
  if (j >= threshold)
    return 0.0;
  return std::clamp(1.0 - j / threshold, 0.0, 1.0);
}

}  // namespace gz::sim::waves

#endif  // GZ_SIM_WAVES_EVAL_HH_
