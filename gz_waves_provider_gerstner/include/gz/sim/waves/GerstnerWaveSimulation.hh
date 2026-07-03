/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef GZ_SIM_WAVES_GERSTNERWAVESIMULATION_HH_
#define GZ_SIM_WAVES_GERSTNERWAVESIMULATION_HH_

#include <memory>
#include <vector>

#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>

#include "gz/sim/waves/WaveSimulation.hh"
#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::waves
{

/// \brief Analytic sum-of-Gerstners wave field engine. Closed-form for each of up
/// to N component waves; deterministic; unbounded in space.
///
/// \note Kinematics differ by consumer. The point queries (`Elevation`,
/// `ParticleVelocity`, `Normal`), i.e. the physics path buoyancy reads, use
/// linear (Airy) kinematics: they sample the vertical height field Σ a·cos(θ)
/// directly at the query (x, y), with no horizontal displacement. The render grid
/// (`Update`/`Field`, consumed by the shader) additionally applies the full
/// Gerstner horizontal chop (dx/dy = −q·a·dir·sin(θ)), so the *visible* surface is
/// trochoidal. Inverting the Gerstner parametric map per query would be costly and
/// is unnecessary for buoyancy, where the vertical profile dominates; the chop is
/// a visual refinement.
class GerstnerWaveSimulation final : public IWaveField
{
  /// \brief Default-construct an unconfigured field. Call `SetParameters`
  /// before sampling. Used by the engine factory (`MakeGerstnerWaveField`).
  public: GerstnerWaveSimulation();

  /// \brief Construct and sample the spectrum from `_params` (convenience;
  /// equivalent to default-construct followed by `SetParameters`).
  /// \param[in] _params Wave parameters to build the field from.
  public: explicit GerstnerWaveSimulation(const WaveParameters &_params);

  /// \brief Destructor.
  public: ~GerstnerWaveSimulation() override = default;

  // Documentation inherited
  public: void SetParameters(const WaveParameters &_params) override;
  // Documentation inherited
  public: double Elevation(double _x, double _y, double _t) const override;
  // Documentation inherited
  public: gz::math::Vector3d ParticleVelocity(
    double _x, double _y, double _t) const override;
  // Documentation inherited
  public: gz::math::Vector3d Normal(
    double _x, double _y, double _t) const override;
  // Documentation inherited
  public: double Jacobian(double _x, double _y, double _t) const override;
  // Documentation inherited
  public: void Update(double _simTime) override;
  // Documentation inherited
  public: const WaveField2D *Field() const override;
  // Documentation inherited
  public: std::string_view Kind() const override { return "gerstner"; }

  // ---- Engine-specific introspection accessors (exercised by the unit
  //      tests; not part of the IWaveField interface — WaterVisual consumes
  //      Field() instead) ----

  /// \brief Per-component wave amplitudes [m].
  public: const std::vector<double> &Amplitudes() const
  {
    return this->amplitudes;
  }
  /// \brief Per-component wavenumbers |k| [rad/m].
  public: const std::vector<double> &Wavenumbers() const
  {
    return this->wavenumbers;
  }
  /// \brief Per-component angular frequencies ω [rad/s].
  public: const std::vector<double> &AngularFrequencies() const
  {
    return this->angularFrequencies;
  }
  /// \brief Per-component Gerstner steepness in [0, 1].
  public: const std::vector<double> &Steepnesses() const
  {
    return this->steepnesses;
  }
  /// \brief Per-component unit propagation directions.
  public: const std::vector<gz::math::Vector2d> &Directions() const
  {
    return this->directions;
  }

  /// \brief Phase θ_i(x, y, t) = k_i·(d_i·(x, y)) − ω_i·t + φ of component `_i`
  /// at world point (x, y) and time t. Shared by every per-component sum
  /// (Elevation / ParticleVelocity / Normal / Jacobian / the render grid) so
  /// the phase is computed one way only.
  /// \param[in] _i Component index (< number of components).
  /// \param[in] _x World-frame x coordinate [m].
  /// \param[in] _y World-frame y coordinate [m].
  /// \param[in] _t Simulation time [s].
  /// \return The component's phase [rad].
  private: double Phase(std::size_t _i, double _x, double _y, double _t) const;

  /// \brief Per-component wave amplitudes [m].
  private: std::vector<double> amplitudes;
  /// \brief Per-component wavenumbers |k| [rad/m].
  private: std::vector<double> wavenumbers;
  /// \brief Per-component angular frequencies ω [rad/s].
  private: std::vector<double> angularFrequencies;
  /// \brief Per-component Gerstner steepness in [0, 1].
  private: std::vector<double> steepnesses;
  /// \brief Per-component unit propagation directions.
  private: std::vector<gz::math::Vector2d> directions;
  /// \brief Startup-ramp time constant τ [s].
  private: double tau{2.0};
  /// \brief Common phase offset φ [rad].
  private: double phase{0.0};

  // Render grid: the analytic field sampled onto an N×N tile each Update,
  // exposed via Field() as the engine-agnostic rendering contract. Buffers
  // are column-major; Update overwrites them in place, so field's data()
  // pointers (bound in SetParameters) stay valid.

  /// \brief Render-grid resolution per axis (N).
  private: std::size_t fieldN{128};
  /// \brief Render-grid tile extent [m].
  private: double fieldTile{0.0};
  /// \brief Sim time the render grid was last sampled at.
  private: double currentTime{-1.0};
  /// \brief Column-major vertical-displacement buffer for `field.dz`.
  private: std::vector<double> dzBuf;
  /// \brief Column-major x-chop buffer for `field.dx`.
  private: std::vector<double> dxBuf;
  /// \brief Column-major y-chop buffer for `field.dy`.
  private: std::vector<double> dyBuf;
  /// \brief Column-major folding/foam buffer for `field.foam`.
  private: std::vector<double> foamBuf;
  /// \brief The rendering contract returned by Field().
  private: WaveField2D field;
};

/// \brief Factory: a default-constructed Gerstner wave-field engine (apply
/// `SetParameters` before use). Registered under the "gerstner" token so
/// `CreateWaveSimulation` can rebuild the engine from a serialized `Wavefield`
/// component on the GUI side.
std::shared_ptr<IWaveField> MakeGerstnerWaveField();

}  // namespace gz::sim::waves

#endif  // GZ_SIM_WAVES_GERSTNERWAVESIMULATION_HH_
