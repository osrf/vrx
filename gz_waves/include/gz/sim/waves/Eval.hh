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

#include <gz/math/Vector3.hh>

#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::waves
{

/// \brief Surface elevation η(x, y, t) [m] above the still water level.
inline double SurfaceElevation(const WavefieldData &wf,
                               double x, double y, double t)
{
  return wf.simulation ? wf.simulation->Elevation(x, y, t) : 0.0;
}

/// \brief Water particle velocity at the surface point (x, y) [m/s].
inline gz::math::Vector3d ParticleVelocity(const WavefieldData &wf,
                                           double x, double y, double t)
{
  return wf.simulation ? wf.simulation->ParticleVelocity(x, y, t)
                       : gz::math::Vector3d::Zero;
}

/// \brief Outward-pointing unit surface normal at (x, y, t).
inline gz::math::Vector3d Normal(const WavefieldData &wf,
                                 double x, double y, double t)
{
  return wf.simulation ? wf.simulation->Normal(x, y, t)
                       : gz::math::Vector3d::UnitZ;
}

/// \brief Jacobian determinant of the horizontal displacement field.
inline double Jacobian(const WavefieldData &wf, double x, double y, double t)
{
  return wf.simulation ? wf.simulation->Jacobian(x, y, t) : 1.0;
}

/// \brief Foam intensity in [0, 1] derived from the Jacobian (Tessendorf 2004).
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
