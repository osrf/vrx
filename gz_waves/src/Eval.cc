/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include "gz/sim/waves/Eval.hh"

#include <algorithm>

#include "gz/sim/waves/WaveSimulation.hh"

namespace gz::sim::waves
{

//////////////////////////////////////////////////
void Advance(WavefieldData &_wf, double _t)
{
  if (_wf.simulation)
    _wf.simulation->Update(_t);
}

//////////////////////////////////////////////////
double SurfaceElevation(const WavefieldData &_wf,
                        double _x, double _y, double _t)
{
  return _wf.simulation ? _wf.simulation->Elevation(_x, _y, _t) : 0.0;
}

//////////////////////////////////////////////////
gz::math::Vector3d ParticleVelocity(const WavefieldData &_wf,
                                    double _x, double _y, double _t)
{
  return _wf.simulation ? _wf.simulation->ParticleVelocity(_x, _y, _t)
                        : gz::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
gz::math::Vector3d Normal(const WavefieldData &_wf,
                          double _x, double _y, double _t)
{
  return _wf.simulation ? _wf.simulation->Normal(_x, _y, _t)
                        : gz::math::Vector3d::UnitZ;
}

//////////////////////////////////////////////////
double Jacobian(const WavefieldData &_wf, double _x, double _y, double _t)
{
  return _wf.simulation ? _wf.simulation->Jacobian(_x, _y, _t) : 1.0;
}

//////////////////////////////////////////////////
double FoamMask(const WavefieldData &_wf, double _x, double _y, double _t,
                double _threshold)
{
  const double j = Jacobian(_wf, _x, _y, _t);
  if (j >= _threshold)
    return 0.0;
  return std::clamp(1.0 - j / _threshold, 0.0, 1.0);
}

}  // namespace gz::sim::waves
