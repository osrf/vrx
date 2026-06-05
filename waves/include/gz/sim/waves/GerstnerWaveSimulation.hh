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

#include <vector>

#include <gz/math/Vector2.hh>

#include "gz/sim/waves/WaveSimulation.hh"
#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::waves
{

/// \brief Analytic sum-of-Gerstners wave model. Closed-form for each of up
/// to N component waves; deterministic; unbounded in space.
class GerstnerWaveSimulation : public IWaveField
{
public:
  /// \brief Construct and sample the spectrum from `_params`.
  explicit GerstnerWaveSimulation(const WaveParameters &_params);

  ~GerstnerWaveSimulation() override = default;

  // IWaveField
  double Elevation(double x, double y, double t) const override;
  gz::math::Vector3d ParticleVelocity(
    double x, double y, double t) const override;
  gz::math::Vector3d Normal(double x, double y, double t) const override;
  double Jacobian(double x, double y, double t) const override;
  std::string_view Kind() const override { return "gerstner"; }

  // ---- Backend-specific accessors (used by WaterVisual to drive shader
  //      uniforms; not part of the IWaveField interface) ----

  const std::vector<double>            &Amplitudes()         const { return amplitudes_; }
  const std::vector<double>            &Wavenumbers()        const { return wavenumbers_; }
  const std::vector<double>            &AngularFrequencies() const { return angularFrequencies_; }
  const std::vector<double>            &Steepnesses()        const { return steepnesses_; }
  const std::vector<gz::math::Vector2d> &Directions()        const { return directions_; }
  double Tau() const { return tau_; }

private:
  /// \brief Startup ramp factor `(1 - exp(-t/tau))`, clamped to [0, 1].
  double Ramp(double t) const;

  std::vector<double>             amplitudes_;
  std::vector<double>             wavenumbers_;
  std::vector<double>             angularFrequencies_;
  std::vector<double>             steepnesses_;
  std::vector<gz::math::Vector2d> directions_;
  double                          tau_{2.0};
};

}  // namespace gz::sim::waves

#endif  // GZ_SIM_WAVES_GERSTNERWAVESIMULATION_HH_
