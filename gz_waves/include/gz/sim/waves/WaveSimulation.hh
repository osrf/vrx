/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef GZ_SIM_WAVES_WAVESIMULATION_HH_
#define GZ_SIM_WAVES_WAVESIMULATION_HH_

#include <cstddef>
#include <functional>
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
  /// \brief Tile extent along x in metres.
  double x{0.0};

  /// \brief Tile extent along y in metres. The tile is centred at the world
  /// origin; samples outside (-x/2, +x/2) × (-y/2, +y/2) are interpreted modulo
  /// the tile size.
  double y{0.0};
};

/// \brief A wave field sampled on a regular N×N grid over one tile — the
/// uniform rendering contract every backend exposes via `IWaveField::Field`.
/// Grid backends fill it from their IFFT output; analytic backends sample
/// themselves into it. The renderer uploads it to a displacement texture and
/// draws one backend-agnostic surface, so it never needs to know which backend
/// is active.
///
/// All arrays are column-major N×N — element (i, j) is at index `i + j*N` —
/// and periodic over the tile. The pointers are owned by the wave-field
/// implementation and remain valid until the next `SetParameters`; the values
/// they reference are refreshed by `Update`.
struct WaveField2D
{
  /// \brief Grid resolution per axis (N).
  std::size_t n{0};

  /// \brief Tile extent [m]; cell spacing is tile/N.
  double tile{0.0};

  /// \brief Vertical displacement η [m] (required).
  const double *dz{nullptr};

  /// \brief Final horizontal displacement x [m]; null ⇒ treat as 0. "Final"
  /// means the engine applies any choppiness/steepness scaling itself —
  /// consumers (the renderer) add dx/dy to the surface as-is. (Engines used
  /// to disagree on this: Gerstner baked the signed chop while the FFT
  /// engine relied on a shader-side chopFactor, so the shared shader
  /// inverted Gerstner's chop.)
  const double *dx{nullptr};

  /// \brief Final horizontal displacement y [m]; null ⇒ treat as 0. See dx.
  const double *dy{nullptr};

  /// \brief Folding metric: 1 = flat, < 1 → folding (whitecaps); null ⇒ the
  /// backend has no foam.
  const double *foam{nullptr};
};

/// \brief Polymorphic backend for a wave field. Concrete implementations (an
/// analytic, closed-form engine; a grid-based, stochastic one; …) live in their
/// own packages and are reached by token via the registry below. Consumers use
/// `Eval::*` free functions which delegate to the implementation; they don't
/// need to know which one is active. The renderer consumes the field uniformly
/// through `Field`.
class IWaveField
{
  /// \brief Virtual destructor.
  public: virtual ~IWaveField() = default;

  // ---- Point queries (mandatory) ------------------------------------------

  /// \brief Surface elevation η(x, y, t) [m] above the still water level.
  /// \param[in] _x World-frame x coordinate [m].
  /// \param[in] _y World-frame y coordinate [m].
  /// \param[in] _t Simulation time [s].
  /// \return Surface elevation above the still water level [m].
  public: virtual double Elevation(double _x, double _y, double _t) const = 0;

  /// \brief Water particle velocity at the surface point (x, y) [m/s].
  /// Used by drag terms for relative-velocity hydrodynamics.
  /// \param[in] _x World-frame x coordinate [m].
  /// \param[in] _y World-frame y coordinate [m].
  /// \param[in] _t Simulation time [s].
  /// \return Water particle velocity at the surface point [m/s].
  public: virtual gz::math::Vector3d ParticleVelocity(
    double _x, double _y, double _t) const = 0;

  /// \brief Outward-pointing unit surface normal at (x, y, t).
  /// \param[in] _x World-frame x coordinate [m].
  /// \param[in] _y World-frame y coordinate [m].
  /// \param[in] _t Simulation time [s].
  /// \return Outward-pointing unit surface normal (dimensionless).
  public: virtual gz::math::Vector3d Normal(
    double _x, double _y, double _t) const = 0;

  /// \brief Jacobian determinant of the horizontal displacement field at
  /// (x, y, t). Values below ~0.6 indicate wave folding / whitecap formation
  /// (Tessendorf 2004).
  /// \param[in] _x World-frame x coordinate [m].
  /// \param[in] _y World-frame y coordinate [m].
  /// \param[in] _t Simulation time [s].
  /// \return Jacobian determinant (dimensionless; 1 = unfolded).
  public: virtual double Jacobian(double _x, double _y, double _t) const = 0;

  // ---- Configuration ------------------------------------------------------

  /// \brief (Re)configure the wave field from `_params`. The engine factory
  /// default-constructs an implementation and then calls this to set it up; it
  /// may also be invoked later to retune parameters at runtime. Implementations
  /// fully (re)build their internal state from `_params`.
  /// \param[in] _params Wave parameters to (re)build the field from.
  public: virtual void SetParameters(const WaveParameters &_params) = 0;

  // ---- Lifecycle ----------------------------------------------------------

  /// \brief Advance any backend-internal time-dependent state. Analytic
  /// backends are stateless and override this as a no-op; grid-based backends
  /// regenerate the height grid here.
  /// \param[in] _simTime Simulation time [s] to advance the field to.
  public: virtual void Update(double /*_simTime*/) {}

  // ---- Backend identity / capabilities ------------------------------------

  /// \brief Short name identifying the backend (e.g. "gerstner", "fft"). Used
  /// by the visual plugin to dispatch to the right shader path.
  /// \return The backend's registration token.
  public: virtual std::string_view Kind() const = 0;

  /// \brief Spatial extent of the wave field. Grid backends return their tile
  /// size; consumers must wrap queries outside the tile.
  /// \return The tile extent, or `nullopt` for a field defined everywhere
  /// (analytic backends).
  public: virtual std::optional<TileSize> Bounds() const
  {
    return std::nullopt;
  }

  // ---- Rendering ----------------------------------------------------------

  /// \brief A view of the current wave field sampled on a grid, for the
  /// renderer to upload as a displacement texture (see `WaveField2D`). The
  /// returned pointer references implementation-owned storage refreshed by
  /// `Update`; it stays valid until the next `SetParameters`.
  /// \return The renderable grid, or `nullptr` if the backend exposes none.
  public: virtual const WaveField2D *Field() const { return nullptr; }
};

/// \brief Builds a default-constructed wave-field engine for a token. The
/// returned engine has NOT yet had `SetParameters` called on it — that is the
/// caller's job (see `CreateWaveSimulation`, which does both). Returning a bare
/// engine keeps the factory free of the `WaveParameters` definition.
using WaveEngineFactory =
  std::function<std::shared_ptr<IWaveField>()>;

/// \brief Register `_factory` under `_token` (e.g. "gerstner", "fft") in the
/// process-wide engine registry that `CreateWaveSimulation` consults.
///
/// This is how a concrete engine library is made reachable by token without
/// the core linking it (which would be a dependency cycle): the package that
/// links an engine — a system plugin on the server, the water visual on the
/// GUI — registers it here. Re-registering a token replaces the prior factory.
/// Thread-safe.
/// \param[in] _token   Engine identifier (matches `IWaveField::Kind`).
/// \param[in] _factory Factory that builds a default-constructed engine.
void RegisterWaveEngineFactory(const std::string &_token,
                               WaveEngineFactory _factory);

/// \brief Construct and configure the wave-field engine named by `_algorithm`
/// (e.g. "gerstner", "fft"): look the token up in the registry populated by
/// `RegisterWaveEngineFactory`, build an engine, and apply `_params` via
/// `SetParameters`.
/// \param[in] _algorithm Engine token to construct.
/// \param[in] _params    Parameters to configure the new engine with.
/// \return The configured engine, or `nullptr` for an unregistered token.
std::shared_ptr<IWaveField> CreateWaveSimulation(
  const std::string &_algorithm,
  const WaveParameters &_params);

}  // namespace gz::sim::waves

#endif  // GZ_SIM_WAVES_WAVESIMULATION_HH_
