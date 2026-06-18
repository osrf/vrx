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

#include <Eigen/Core>

#include "gz/sim/waves/WaveSimulation.hh"
#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::waves
{

/// \brief Analytic sum-of-Gerstners wave model. Closed-form for each of up
/// to N component waves; deterministic; unbounded in space.
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
  public: Eigen::Vector3d ParticleVelocity(
    double _x, double _y, double _t) const override;
  // Documentation inherited
  public: Eigen::Vector3d Normal(
    double _x, double _y, double _t) const override;
  // Documentation inherited
  public: double Jacobian(double _x, double _y, double _t) const override;
  // Documentation inherited
  public: void Update(double _simTime) override;
  // Documentation inherited
  public: const WaveField2D *Field() const override;
  // Documentation inherited
  public: std::string_view Kind() const override { return "gerstner"; }

  // ---- Backend-specific introspection accessors (exercised by the unit
  //      tests; not part of the IWaveField interface — WaterVisual consumes
  //      Field() instead) ----

  /// \brief Per-component wave amplitudes [m].
  public: const std::vector<double> &Amplitudes() const
  {
    return this->amplitudes_;
  }
  /// \brief Per-component wavenumbers |k| [rad/m].
  public: const std::vector<double> &Wavenumbers() const
  {
    return this->wavenumbers_;
  }
  /// \brief Per-component angular frequencies ω [rad/s].
  public: const std::vector<double> &AngularFrequencies() const
  {
    return this->angularFrequencies_;
  }
  /// \brief Per-component Gerstner steepness in [0, 1].
  public: const std::vector<double> &Steepnesses() const
  {
    return this->steepnesses_;
  }
  /// \brief Per-component unit propagation directions.
  public: const std::vector<Eigen::Vector2d> &Directions() const
  {
    return this->directions_;
  }

  /// \brief Per-component wave amplitudes [m].
  private: std::vector<double> amplitudes_;
  /// \brief Per-component wavenumbers |k| [rad/m].
  private: std::vector<double> wavenumbers_;
  /// \brief Per-component angular frequencies ω [rad/s].
  private: std::vector<double> angularFrequencies_;
  /// \brief Per-component Gerstner steepness in [0, 1].
  private: std::vector<double> steepnesses_;
  /// \brief Per-component unit propagation directions.
  private: std::vector<Eigen::Vector2d> directions_;
  /// \brief Startup-ramp time constant τ [s].
  private: double tau_{2.0};
  /// \brief Common phase offset φ [rad].
  private: double phase_{0.0};

  // Render grid: the analytic field sampled onto an N×N tile each Update,
  // exposed via Field() as the backend-agnostic rendering contract. Buffers
  // are column-major; Update overwrites them in place, so field_'s data()
  // pointers (bound in SetParameters) stay valid.

  /// \brief Render-grid resolution per axis (N).
  private: std::size_t fieldN_{128};
  /// \brief Render-grid tile extent [m].
  private: double fieldTile_{0.0};
  /// \brief Sim time the render grid was last sampled at.
  private: double currentTime_{-1.0};
  /// \brief Column-major vertical-displacement buffer for `field_.dz`.
  private: std::vector<double> dzBuf_;
  /// \brief Column-major x-chop buffer for `field_.dx`.
  private: std::vector<double> dxBuf_;
  /// \brief Column-major y-chop buffer for `field_.dy`.
  private: std::vector<double> dyBuf_;
  /// \brief Column-major folding/foam buffer for `field_.foam`.
  private: std::vector<double> foamBuf_;
  /// \brief The rendering contract returned by Field().
  private: WaveField2D field_;
};

/// \brief Factory: a default-constructed Gerstner wave-field engine (apply
/// `SetParameters` before use). Registered under the "gerstner" token so
/// `CreateWaveSimulation` can rebuild the engine from a serialized `Wavefield`
/// component on the GUI side.
std::shared_ptr<IWaveField> MakeGerstnerWaveField();

}  // namespace gz::sim::waves

#endif  // GZ_SIM_WAVES_GERSTNERWAVESIMULATION_HH_
