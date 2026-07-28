/*
 * Copyright (C) 2026 Honu Robotics
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
/// \brief Wave source system backed by the stochastic FFT engine — the
/// Apache-2.0 EncinoWaves spectral library (TMA/JONSWAP/PM spectra, default
/// TMA + Hasselmann spreading + capillary dispersion). Loaded by SDF as
/// `gz-sim-waves-fft-system`. The ECM/source plumbing lives in WavesSystemBase;
/// this names the engine and owns the FFT parameter surface below.
///
/// ## SDF parameters
///
/// `<update_rate>` is at plugin level; the rest live in a `<wave>` block. All
/// are also runtime-settable via the `/world/<name>/wave/set_parameters`
/// service (`gz.msgs.Param`).
///
/// | Tag | Type | Default | Meaning |
/// |---|---|---|---|
/// | `<update_rate>` | double [Hz] | 30.0 | Throttle for the FFT recompute (plugin level) |
/// | `<period>` | double [s] | 5.0 | Wave period -> wind speed (PM relation) |
/// | `<gain>` | double | 1.0 | Amplitude multiplier |
/// | `<tau>` | double [s] | 2.0 | Startup-ramp time constant, (1 - exp(-t/tau)) |
/// | `<sea_state>` | int [0-9] | -1 (off) | WMO sea-state code; when >= 0 it OVERRIDES `<period>`/`<gain>` — see "Sea state" below |
/// | `<tile_size>` | double [m] | 200.0 | Periodic tile extent per axis |
/// | `<grid_size>` | uint | 128 | Grid samples/axis (power of two; rounded up) |
/// | `<seed>` | uint | 0 | RNG seed for the spectrum amplitudes |
/// | `<choppiness>` | double | -1.0 | Tessendorf horizontal-displacement multiplier (~[-2, 0]); applied in the visual shader |
/// | `<spectrum>` | string | tma | EncinoWaves spectrum: `pms`, `jonswap`, `tma` |
/// | `<spreading>` | string | hasselmann | Directional spreading: `poscos2`, `mitsuyasu`, `hasselmann`, `donelanbanner` |
/// | `<dispersion>` | string | capillary | Dispersion relation: `deep`, `finite`, `capillary` |
/// | `<depth>` | double [m] | 100 | Water depth (EncinoWaves dispersion input) |
/// | `<fetch>` | double [km] | 300 | Wind fetch (EncinoWaves spectrum input) |
/// | `<swell>` | double | 0 | Swell elongation (directional spreading) |
/// | `<trough_damping>` | double [0,1] | 0 | Breaking-wave trough damping |
/// | `<filter_min_wl>` | double [m] | 0 | Band-pass lower edge; >0 enables the filter |
/// | `<filter_max_wl>` | double [m] | 0 | Band-pass upper edge (0 = no upper bound) |
/// | `<filter_soft>` | double [m] | 0 | Band-pass transition width (0 = auto) |
/// | `<filter_min>` | double [0,1] | 0 | Band-pass suppression floor |
/// | `<filter_invert>` | bool | false | Band-stop (notch) instead of band-pass |
///
/// ### Sea state vs. manual height
///
/// `<sea_state>` is the one-knob control: a WMO code 0-9 (table 3700) that sets
/// the sea by deriving its significant wave height and peak period. Precedence:
/// - When `>= 0` it **overrides `<period>` and `<gain>`** — setting those
///   alongside it has no effect, so use one approach or the other.
/// - `0` is calm/glassy (flat) water; `-1` (the default, or simply omitting the
///   tag) means "manual": drive the sea with `<period>` + `<gain>` instead.
/// - Every other knob (`<direction>`, `<tile_size>`, `<choppiness>`, the
///   spectrum/spreading/dispersion tags, ...) is independent of `<sea_state>`
///   and applies in both modes — combine freely.
///
/// `<direction>` is parsed but not applied — EncinoWaves assumes wind along +X.
///
/// \verbatim
/// <plugin filename="gz-sim-waves-fft-system" name="gz::sim::systems::FftWaves">
///   <update_rate>30</update_rate>
///   <wave>
///     <!-- <sea_state>5</sea_state>  optional WMO 0-9; overrides period+gain -->
///     <period>3.2</period>
///     <gain>1.0</gain>
///     <tau>2.0</tau>
///     <tile_size>256</tile_size>
///     <grid_size>128</grid_size>
///     <seed>42</seed>
///     <choppiness>-2.0</choppiness>
///     <spectrum>tma</spectrum>
///     <spreading>hasselmann</spreading>
///     <dispersion>capillary</dispersion>
///   </wave>
/// </plugin>
/// \endverbatim
class FftWaves : public WavesSystemBase
{
  // Documentation inherited
  protected: std::string EngineToken() const override { return "fft"; }

  // Documentation inherited
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
              gz::sim::systems::FftWaves::ISystemPreUpdate,
              gz::sim::systems::FftWaves::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::FftWaves, "gz::sim::systems::FftWaves")
