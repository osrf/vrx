/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
public:
  /// \brief Default-construct an unconfigured field. Call `SetParameters`
  /// before sampling. Used by the engine factory (`MakeGerstnerWaveField`).
  GerstnerWaveSimulation();

  /// \brief Construct and sample the spectrum from `_params` (convenience;
  /// equivalent to default-construct followed by `SetParameters`).
  explicit GerstnerWaveSimulation(const WaveParameters &_params);

  /// \brief Destructor.
  ~GerstnerWaveSimulation() override = default;

  // IWaveField
  void SetParameters(const WaveParameters &_params) override;
  double Elevation(double x, double y, double t) const override;
  Eigen::Vector3d ParticleVelocity(
    double x, double y, double t) const override;
  Eigen::Vector3d Normal(double x, double y, double t) const override;
  double Jacobian(double x, double y, double t) const override;
  void Update(double _simTime) override;
  const WaveField2D *Field() const override;
  std::string_view Kind() const override { return "gerstner"; }

  // ---- Backend-specific introspection accessors (exercised by the unit
  //      tests; not part of the IWaveField interface — WaterVisual consumes
  //      Field() instead) ----

  /// \brief Per-component wave amplitudes [m].
  const std::vector<double>            &Amplitudes()         const { return amplitudes_; }
  /// \brief Per-component wavenumbers |k| [rad/m].
  const std::vector<double>            &Wavenumbers()        const { return wavenumbers_; }
  /// \brief Per-component angular frequencies ω [rad/s].
  const std::vector<double>            &AngularFrequencies() const { return angularFrequencies_; }
  /// \brief Per-component Gerstner steepness in [0, 1].
  const std::vector<double>            &Steepnesses()        const { return steepnesses_; }
  /// \brief Per-component unit propagation directions.
  const std::vector<Eigen::Vector2d> &Directions()        const { return directions_; }

private:
  std::vector<double>             amplitudes_;
  std::vector<double>             wavenumbers_;
  std::vector<double>             angularFrequencies_;
  std::vector<double>             steepnesses_;
  std::vector<Eigen::Vector2d>    directions_;
  double                          tau_{2.0};
  double                          phase_{0.0};  ///< Common phase offset φ [rad].

  // Render grid: the analytic field sampled onto an N×N tile each Update,
  // exposed via Field() as the backend-agnostic rendering contract. Buffers
  // are column-major; Update overwrites them in place, so field_'s data()
  // pointers (bound in SetParameters) stay valid.
  std::size_t         fieldN_{128};
  double              fieldTile_{0.0};
  double              currentTime_{-1.0};
  std::vector<double> dzBuf_;
  std::vector<double> dxBuf_;
  std::vector<double> dyBuf_;
  std::vector<double> foamBuf_;
  WaveField2D         field_;
};

/// \brief Factory: a default-constructed Gerstner wave-field engine (apply
/// `SetParameters` before use). Registered under the "gerstner" token so
/// `CreateWaveSimulation` can rebuild the engine from a serialized `Wavefield`
/// component on the GUI side.
std::shared_ptr<IWaveField> MakeGerstnerWaveField();

}  // namespace gz::sim::waves

#endif  // GZ_SIM_WAVES_GERSTNERWAVESIMULATION_HH_
