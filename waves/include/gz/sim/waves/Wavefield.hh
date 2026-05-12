/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef GZ_SIM_WAVES_WAVEFIELD_HH_
#define GZ_SIM_WAVES_WAVEFIELD_HH_

#include <cstddef>
#include <cstdint>
#include <istream>
#include <ostream>
#include <string>
#include <vector>

#include <gz/math/Vector2.hh>

namespace gz::sim::waves
{

/// \brief Human-facing wave parameters. Set via SDF, scenarios, or runtime API.
///
/// Two spectrum families are supported, selected by `model`:
///   * "PMS" - Pierson-Moskowitz spectrum sampling. Amplitudes derive from the
///             spectrum at frequencies spaced around the mean.
///   * "CWR" - Constant Wavelength Ratio. Parametric: amplitudes scale with
///             the mean amplitude; wavenumbers scale with the mean.
struct WaveParameters
{
  /// \brief Spectrum / model selector. "PMS" or "CWR".
  std::string model{"PMS"};

  /// \brief Number of component waves summed in the Gerstner expansion.
  std::size_t number{3};

  /// \brief Mean wave period [s].
  double period{5.0};

  /// \brief Mean wave amplitude [m] (CWR only).
  double amplitude{0.0};

  /// \brief Mean wave direction [rad] (angle from +X in the world frame).
  double direction{0.0};

  /// \brief Angle between mean direction and largest/smallest component [rad].
  double angle{0.4};

  /// \brief Scale between mean and largest/smallest component waves.
  double scale{1.1};

  /// \brief Wave steepness in [0, 1]. 0 = sine waves; 1 = max Gerstner.
  double steepness{0.0};

  /// \brief Wave phase [rad] (common to all components).
  double phase{0.0};

  /// \brief Time constant for the startup ramp `(1 - exp(-t/tau))` [s].
  double tau{2.0};

  /// \brief PMS amplitude multiplier (unitless).
  double gain{1.0};
};

/// \brief Wave field state held by the `Wavefield` ECM component.
///
/// Holds the input parameters plus the derived component arrays (amplitudes,
/// wavenumbers, frequencies, directions, steepnesses). The `Waves` system
/// owns writes; everyone else queries via free functions in `Eval.hh`.
///
/// Convention: the component arrays are populated by
/// `gz::sim::waves::SampleSpectrum(data)`. The `generation` counter is bumped
/// whenever the data is rewritten so consumers can detect changes cheaply.
struct WavefieldData
{
  /// \brief Input parameters.
  WaveParameters params;

  /// \brief Component-wave amplitudes [m].
  std::vector<double> amplitudes;

  /// \brief Component-wave numbers [rad/m].
  std::vector<double> wavenumbers;

  /// \brief Component-wave angular frequencies [rad/s].
  std::vector<double> angularFrequencies;

  /// \brief Component Gerstner steepnesses (q) in [0, 1].
  std::vector<double> steepnesses;

  /// \brief Component propagation directions (unit 2D vectors in world frame).
  std::vector<gz::math::Vector2d> directions;

  /// \brief Monotonically increasing counter, bumped on every rewrite.
  std::uint64_t generation{0};
};

/// \brief Stream-out for ECM serialization (SceneBroadcaster → GUI).
inline std::ostream &operator<<(std::ostream &_os, const WavefieldData &_d)
{
  _os << _d.params.model << ' '
      << _d.params.number << ' '
      << _d.params.period << ' '
      << _d.params.amplitude << ' '
      << _d.params.direction << ' '
      << _d.params.angle << ' '
      << _d.params.scale << ' '
      << _d.params.steepness << ' '
      << _d.params.phase << ' '
      << _d.params.tau << ' '
      << _d.params.gain << ' '
      << _d.generation << ' '
      << _d.amplitudes.size() << ' ';
  for (auto v : _d.amplitudes)         _os << v << ' ';
  for (auto v : _d.wavenumbers)        _os << v << ' ';
  for (auto v : _d.angularFrequencies) _os << v << ' ';
  for (auto v : _d.steepnesses)        _os << v << ' ';
  for (const auto &d : _d.directions)  _os << d.X() << ' ' << d.Y() << ' ';
  return _os;
}

/// \brief Stream-in for ECM deserialization.
inline std::istream &operator>>(std::istream &_is, WavefieldData &_d)
{
  _is >> _d.params.model
      >> _d.params.number
      >> _d.params.period
      >> _d.params.amplitude
      >> _d.params.direction
      >> _d.params.angle
      >> _d.params.scale
      >> _d.params.steepness
      >> _d.params.phase
      >> _d.params.tau
      >> _d.params.gain
      >> _d.generation;
  std::size_t n = 0;
  _is >> n;
  _d.amplitudes.resize(n);
  _d.wavenumbers.resize(n);
  _d.angularFrequencies.resize(n);
  _d.steepnesses.resize(n);
  _d.directions.resize(n);
  for (auto &v : _d.amplitudes)         _is >> v;
  for (auto &v : _d.wavenumbers)        _is >> v;
  for (auto &v : _d.angularFrequencies) _is >> v;
  for (auto &v : _d.steepnesses)        _is >> v;
  for (auto &d : _d.directions)
  {
    double x = 0.0, y = 0.0;
    _is >> x >> y;
    d.Set(x, y);
  }
  return _is;
}

}  // namespace gz::sim::waves

#endif  // GZ_SIM_WAVES_WAVEFIELD_HH_
