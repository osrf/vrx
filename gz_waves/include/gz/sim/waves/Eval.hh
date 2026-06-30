/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef GZ_SIM_WAVES_EVAL_HH_
#define GZ_SIM_WAVES_EVAL_HH_

#include <gz/math/Vector3.hh>

#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::waves
{

// The wave field is queried only through these free functions: they take the
// `Wavefield` component's data and forward to the polymorphic engine it holds.
// Keeping them out-of-line (defined in Eval.cc) lets a consumer use the
// component without seeing the full `IWaveField` interface — `Wavefield.hh`
// only forward-declares it. Each is null-safe: with no engine the field reads
// as still water.

/// \brief Advance the wave field held by `_wf` to time `_t` [s] (no-op when no
/// engine is present). Consumers call this instead of touching the engine
/// directly, keeping `IWaveField` opaque to them.
/// \param[in,out] _wf Wave-field state whose engine is advanced.
/// \param[in]     _t  Simulation time [s] to advance to.
void Advance(WavefieldData &_wf, double _t);

/// \brief Surface elevation η(x, y, t) [m] above the still water level.
/// \param[in] _wf Wave-field state to query.
/// \param[in] _x  World-frame x coordinate [m].
/// \param[in] _y  World-frame y coordinate [m].
/// \param[in] _t  Simulation time [s].
/// \return Elevation above the still water level [m]; 0 if `_wf` holds no
/// engine.
double SurfaceElevation(const WavefieldData &_wf,
                        double _x, double _y, double _t);

/// \brief Water particle velocity at the surface point (x, y) [m/s].
/// \param[in] _wf Wave-field state to query.
/// \param[in] _x  World-frame x coordinate [m].
/// \param[in] _y  World-frame y coordinate [m].
/// \param[in] _t  Simulation time [s].
/// \return Particle velocity [m/s]; zero if `_wf` holds no engine.
gz::math::Vector3d ParticleVelocity(const WavefieldData &_wf,
                                 double _x, double _y, double _t);

/// \brief Outward-pointing unit surface normal at (x, y, t).
/// \param[in] _wf Wave-field state to query.
/// \param[in] _x  World-frame x coordinate [m].
/// \param[in] _y  World-frame y coordinate [m].
/// \param[in] _t  Simulation time [s].
/// \return Unit surface normal; +Z if `_wf` holds no engine.
gz::math::Vector3d Normal(const WavefieldData &_wf,
                       double _x, double _y, double _t);

/// \brief Jacobian determinant of the horizontal displacement field.
/// \param[in] _wf Wave-field state to query.
/// \param[in] _x  World-frame x coordinate [m].
/// \param[in] _y  World-frame y coordinate [m].
/// \param[in] _t  Simulation time [s].
/// \return Jacobian determinant; 1 if `_wf` holds no engine.
double Jacobian(const WavefieldData &_wf, double _x, double _y, double _t);

/// \brief Foam intensity in [0, 1] derived from the Jacobian (Tessendorf 2004).
/// \param[in] _wf        Wave-field state to query.
/// \param[in] _x         World-frame x coordinate [m].
/// \param[in] _y         World-frame y coordinate [m].
/// \param[in] _t         Simulation time [s].
/// \param[in] _threshold Jacobian value at/above which foam is zero.
/// \return Foam intensity in [0, 1] (0 = no foam).
double FoamMask(const WavefieldData &_wf, double _x, double _y, double _t,
                double _threshold = 0.6);

}  // namespace gz::sim::waves

#endif  // GZ_SIM_WAVES_EVAL_HH_
