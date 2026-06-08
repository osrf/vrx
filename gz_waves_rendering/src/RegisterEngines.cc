/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

/// \file
/// \brief GUI-side wave-engine registration.
///
/// The water visual runs in the GUI process and reconstructs the wave engine
/// from the serialized `Wavefield` component via `CreateWaveSimulation`
/// (`WavefieldData::operator>>`). Engines are plain libraries now — not
/// runtime-loaded gz-plugins — so this translation unit, compiled into the
/// water-visual system, registers both factories. Referencing the `Make*`
/// functions also forces both engine libraries to be linked (so `--as-needed`
/// can't drop them), and running at library-load time guarantees the registry
/// is populated before the first component deserialization on the GUI side.

#include "gz/sim/waves/WaveSimulation.hh"          // RegisterWaveEngineFactory
#include "gz/sim/waves/FFTWaveSimulation.hh"       // MakeFFTWaveField
#include "gz/sim/waves/GerstnerWaveSimulation.hh"  // MakeGerstnerWaveField

namespace
{
const bool kEnginesRegistered = [] {
  namespace gsw = gz::sim::waves;
  gsw::RegisterWaveEngineFactory("fft", &gsw::MakeFFTWaveField);
  gsw::RegisterWaveEngineFactory("gerstner", &gsw::MakeGerstnerWaveField);
  return true;
}();
}  // namespace
