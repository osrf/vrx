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
#include <memory>
#include <ostream>
#include <string>

#include "gz/sim/waves/WaveSimulation.hh"

namespace gz::sim::waves
{

/// \brief Human-facing wave parameters parsed from SDF or set programmatically.
struct WaveParameters
{
  /// \brief Spectrum / sampling model: "PMS" or "CWR".
  std::string model{"PMS"};

  /// \brief Number of component waves (for Gerstner) or grid samples (for
  /// future FFT layouts).
  std::size_t number{3};

  /// \brief Mean wave period [s].
  double period{5.0};

  /// \brief Mean wave amplitude [m] (CWR model only).
  double amplitude{0.0};

  /// \brief Mean wave direction [rad] from +X.
  double direction{0.0};

  /// \brief Angle between mean direction and largest/smallest component [rad].
  double angle{0.4};

  /// \brief Scale between mean and largest/smallest component waves.
  double scale{1.1};

  /// \brief Wave steepness in [0, 1].
  double steepness{0.0};

  /// \brief Phase [rad] (common to all components).
  double phase{0.0};

  /// \brief Time constant for the startup ramp `(1 - exp(-t/tau))` [s].
  double tau{2.0};

  /// \brief PMS amplitude multiplier (unitless).
  double gain{1.0};

  // ---- FFT-only parameters (ignored by the Gerstner backend) -------------

  /// \brief Physical tile extent [m] along each axis for the periodic
  /// FFT wave field.
  double tileSize{200.0};

  /// \brief Number of grid samples per axis for the FFT. Must be a power of
  /// two (FFT/IFFT requirement). 64/128/256 typical.
  std::size_t gridSize{128};

  /// \brief RNG seed used to generate the stochastic Phillips-spectrum
  /// amplitudes. Same seed → bit-for-bit identical wave field.
  std::uint32_t seed{0};
};

/// \brief State held by the `Wavefield` ECM component. Wraps a polymorphic
/// `IWaveSimulation` (Gerstner today, FFT or others later). Consumers use
/// the free functions in `Eval.hh` to query the wave field; they don't see
/// the backend directly.
struct WavefieldData
{
  /// \brief Backend identifier — "gerstner" or "fft". Serialized so the GUI
  /// process can reconstruct the right backend on deserialization.
  std::string algorithm{"gerstner"};

  /// \brief Inputs to the backend.
  WaveParameters params;

  /// \brief The polymorphic wave-field implementation. Constructed by the
  /// `Waves` system from `algorithm` + `params`. Consumers query via
  /// `Eval::*` free functions.
  std::shared_ptr<IWaveSimulation> simulation;

  /// \brief Monotonically increasing counter, bumped whenever the data is
  /// rewritten. Consumers (visual upload, etc.) compare against their last
  /// seen value to cheaply detect changes.
  std::uint64_t generation{0};
};

/// \brief Stream-out for ECM serialization. Writes the algorithm + the
/// parameter struct. The simulation pointer is not serialized — it's
/// reconstructed on the receiving side via the factory in WaveSimulation.hh.
inline std::ostream &operator<<(std::ostream &_os, const WavefieldData &_d)
{
  _os << _d.algorithm << ' '
      << _d.generation << ' '
      << _d.params.model << ' '
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
      << _d.params.tileSize << ' '
      << _d.params.gridSize << ' '
      << _d.params.seed << ' ';
  return _os;
}

/// \brief Stream-in for ECM deserialization. Reads parameters and rebuilds
/// the simulation via `CreateWaveSimulation`.
inline std::istream &operator>>(std::istream &_is, WavefieldData &_d)
{
  _is >> _d.algorithm
      >> _d.generation
      >> _d.params.model
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
      >> _d.params.tileSize
      >> _d.params.gridSize
      >> _d.params.seed;
  _d.simulation = CreateWaveSimulation(_d.algorithm, _d.params);
  return _is;
}

}  // namespace gz::sim::waves

#endif  // GZ_SIM_WAVES_WAVEFIELD_HH_
