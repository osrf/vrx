/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include <gz/plugin/Register.hh>

#include "gz/sim/System.hh"
#include "gz/sim/waves/FFTWaveSimulation.hh"  // MakeFFTWaveField
#include "gz/sim/waves/WaveSimulation.hh"     // RegisterWaveEngineFactory

namespace gz::sim::systems
{
/// \brief GUI-side registrar for the FFT wave engine. Loaded in the GUI process
/// alongside WaterVisual so the in-process engine registry knows the "fft"
/// token, letting WaterVisual rebuild its own (thread-private) engine from the
/// replicated recipe via `CreateWaveSimulation`. The server builds its engine
/// directly (`FftWaves::MakeEngine`) and does not need this. Keeping
/// registration here — rather than in `gz_waves_rendering` — lets the rendering
/// package stay provider-agnostic (it depends only on the gz_waves core). The
/// GUI-side mirror of the server's `gz-sim-waves-fft-system`.
///
/// The factory is registered by the file-scope initializer below, which runs
/// the moment the GUI dlopens this library — so the plugin deliberately carries
/// no ISystem interface and parses no SDF, avoiding gz-sim's empty-plugin SDF
/// re-parse warning. The empty System body exists only so gz-sim has a plugin
/// to load (which is what triggers the dlopen).
class FftWavesGui : public System
{
};
}  // namespace gz::sim::systems

namespace
{
/// \brief Register the "fft" engine factory when this library is loaded.
const bool kFftGuiRegistered = []
{
  gz::sim::waves::RegisterWaveEngineFactory(
    "fft", &gz::sim::waves::MakeFFTWaveField);
  return true;
}();
}  // namespace

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(gz::sim::systems::FftWavesGui, gz::sim::System)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::FftWavesGui,
                    "gz::sim::systems::FftWavesGui")
