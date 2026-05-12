/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include "gz/sim/waves/WaveSimulation.hh"

#include <iostream>

#include "gz/sim/waves/GerstnerWaveSimulation.hh"
#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::waves
{

std::shared_ptr<IWaveSimulation> CreateWaveSimulation(
  const std::string &_algorithm,
  const WaveParameters &_params)
{
  if (_algorithm == "gerstner")
  {
    return std::make_shared<GerstnerWaveSimulation>(_params);
  }
  // FFT will be wired here at Stage 2.
  std::cerr << "[CreateWaveSimulation] unknown algorithm '"
            << _algorithm << "'; supported: 'gerstner'" << std::endl;
  return nullptr;
}

}  // namespace gz::sim::waves
