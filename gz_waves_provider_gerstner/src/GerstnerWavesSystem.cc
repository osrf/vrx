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
#include "gz/sim/waves/GerstnerWaveSimulation.hh"  // MakeGerstnerWaveField
#include "gz/sim/waves/Wavefield.hh"               // WaveParameters

namespace gz::sim::systems
{
/// \brief Wave source system backed by the analytic Gerstner engine. Loaded by
/// SDF as `gz-sim-waves-gerstner-system`; all the ECM/source plumbing lives in
/// WavesSystemBase, so this only names the engine.
class GerstnerWaves : public WavesSystemBase
{
  protected: std::string EngineToken() const override { return "gerstner"; }

  protected: std::shared_ptr<waves::IWaveField> MakeEngine(
    const waves::WaveParameters &_params) const override
  {
    auto engine = waves::MakeGerstnerWaveField();
    engine->SetParameters(_params);
    return engine;
  }
};
}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::GerstnerWaves,
              gz::sim::System,
              gz::sim::systems::GerstnerWaves::ISystemConfigure,
              gz::sim::systems::GerstnerWaves::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::GerstnerWaves,
                    "gz::sim::systems::GerstnerWaves")
