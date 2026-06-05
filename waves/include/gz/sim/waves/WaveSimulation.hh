/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef GZ_SIM_WAVES_WAVESIMULATION_HH_
#define GZ_SIM_WAVES_WAVESIMULATION_HH_

#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include <gz/math/Vector3.hh>

namespace gz::sim::waves
{

struct WaveParameters;

/// \brief Spatial extent of a wave field. Returned by `IWaveField::Bounds`.
struct TileSize
{
  double x{0.0};  ///< Tile extent along x in metres.
  double y{0.0};  ///< Tile extent along y in metres. The tile is centred at
                  ///< the world origin; samples outside (-x/2, +x/2) ×
                  ///< (-y/2, +y/2) are interpreted modulo the tile size.
};

/// \brief Polymorphic backend for a wave field. Concrete implementations
/// include `GerstnerWaveSimulation` (analytic, closed-form) and
/// `FFTWaveSimulation` (grid-based, stochastic, requires `Update` each tick).
/// Consumers use `Eval::*` free functions which delegate to the
/// implementation; they don't need to know which one is active.
class IWaveField
{
public:
  virtual ~IWaveField() = default;

  // ---- Point queries (mandatory) ------------------------------------------

  /// \brief Surface elevation η(x, y, t) [m] above the still water level.
  virtual double Elevation(double x, double y, double t) const = 0;

  /// \brief Water particle velocity at the surface point (x, y) [m/s].
  /// Used by drag terms for relative-velocity hydrodynamics.
  virtual gz::math::Vector3d ParticleVelocity(
    double x, double y, double t) const = 0;

  /// \brief Outward-pointing unit surface normal at (x, y, t).
  virtual gz::math::Vector3d Normal(double x, double y, double t) const = 0;

  /// \brief Jacobian determinant of the horizontal displacement field at
  /// (x, y, t). Values below ~0.6 indicate wave folding / whitecap formation
  /// (Tessendorf 2004).
  virtual double Jacobian(double x, double y, double t) const = 0;

  // ---- Lifecycle ----------------------------------------------------------

  /// \brief Advance any backend-internal time-dependent state. Analytic
  /// backends (Gerstner) are stateless and override this as a no-op;
  /// grid-based backends (FFT) regenerate the height grid here.
  virtual void Update(double /*simTime*/) {}

  // ---- Backend identity / capabilities ------------------------------------

  /// \brief Short name identifying the backend (e.g. "gerstner", "fft").
  /// Used by the visual plugin to dispatch to the right shader path.
  virtual std::string_view Kind() const = 0;

  /// \brief Spatial extent of the wave field, or `nullopt` if the field is
  /// defined everywhere (analytic backends). Grid backends return their
  /// tile size; consumers must wrap queries outside the tile.
  virtual std::optional<TileSize> Bounds() const { return std::nullopt; }
};

/// \brief Factory: instantiate the backend matching `_algorithm` (currently
/// "gerstner" or "fft"). Returns `nullptr` for unknown algorithms.
std::shared_ptr<IWaveField> CreateWaveSimulation(
  const std::string &_algorithm,
  const WaveParameters &_params);

}  // namespace gz::sim::waves

#endif  // GZ_SIM_WAVES_WAVESIMULATION_HH_
