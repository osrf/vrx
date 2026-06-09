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
/// \brief Wave source system backed by the analytic Gerstner engine (a
/// sum-of-Gerstners trochoidal wave field). Loaded by SDF as
/// `gz-sim-waves-gerstner-system`. The ECM/source plumbing lives in
/// WavesSystemBase; this names the engine and owns the Gerstner parameter
/// surface below. (It ignores the FFT-only tags: tile_size, grid_size, seed,
/// choppiness, spectrum, spreading, dispersion.)
///
/// ## SDF parameters
///
/// `<update_rate>` is at plugin level; the rest live in a `<wave>` block. All
/// are also runtime-settable via the `/world/<name>/wave/set_parameters`
/// service (`gz.msgs.Param`).
///
/// | Tag | Type | Default | Meaning |
/// |---|---|---|---|
/// | `<update_rate>` | double [Hz] | 30.0 | Throttle for the backend `Update()` (plugin level) |
/// | `<model>` | string | PMS | Sampling model: `PMS` (wind-derived components) or `CWR` (constant amplitude) |
/// | `<number>` | uint | 3 | Number of component waves |
/// | `<period>` | double [s] | 5.0 | Mean wave period |
/// | `<amplitude>` | double [m] | 0.0 | Mean amplitude (CWR model only) |
/// | `<direction>` | double [rad] | 0.0 | Mean propagation direction from +X |
/// | `<angle>` | double [rad] | 0.4 | Angular spread between components |
/// | `<scale>` | double | 1.1 | Amplitude/length ratio between mean and extreme components |
/// | `<steepness>` | double [0,1] | 0.0 | Crest sharpness (0 = round sine, 1 = pinched) |
/// | `<phase>` | double [rad] | 0.0 | Common phase offset |
/// | `<tau>` | double [s] | 2.0 | Startup-ramp time constant, (1 - exp(-t/tau)) |
/// | `<gain>` | double | 1.0 | Amplitude multiplier |
/// | `<sea_state>` | int [0-9] | -1 | WMO sea-state code; when >= 0 overrides `<period>` + `<gain>` |
///
/// \verbatim
/// <plugin filename="gz-sim-waves-gerstner-system"
///         name="gz::sim::systems::GerstnerWaves">
///   <update_rate>30</update_rate>
///   <wave>
///     <model>PMS</model>
///     <number>3</number>
///     <period>3.2</period>
///     <direction>2.356</direction>
///     <angle>0.4</angle>
///     <scale>1.1</scale>
///     <steepness>0.5</steepness>
///     <gain>1.0</gain>
///     <tau>2.0</tau>
///   </wave>
/// </plugin>
/// \endverbatim
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
