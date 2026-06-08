/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include <memory>
#include <string>

#include <gz/plugin/Register.hh>

#include "gz/sim/systems/WavesSystemBase.hh"
#include "gz/sim/waves/FFTWaveSimulation.hh"  // MakeFFTWaveField
#include "gz/sim/waves/Wavefield.hh"          // WaveParameters

namespace gz::sim::systems
{
/// \brief Wave source system backed by the stochastic FFT engine — Phillips by
/// default, or the EncinoWaves spectra when built with encino and run with
/// GZ_WAVES_USE_ENCINO=1. Loaded by SDF as `gz-sim-waves-fft-system`; all the
/// ECM/source plumbing lives in WavesSystemBase, so this only names the engine.
class FftWaves : public WavesSystemBase
{
  protected: std::string EngineToken() const override { return "fft"; }

  protected: std::shared_ptr<waves::IWaveField> MakeEngine(
    const waves::WaveParameters &_params) const override
  {
    auto engine = waves::MakeFFTWaveField();
    engine->SetParameters(_params);
    return engine;
  }
};
}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::FftWaves,
              gz::sim::System,
              gz::sim::systems::FftWaves::ISystemConfigure,
              gz::sim::systems::FftWaves::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::FftWaves, "gz::sim::systems::FftWaves")
