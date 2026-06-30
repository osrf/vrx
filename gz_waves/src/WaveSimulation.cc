/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include "gz/sim/waves/WaveSimulation.hh"

#include <map>
#include <mutex>
#include <string>
#include <utility>

#include <gz/common/Console.hh>

#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::waves
{

namespace
{
//////////////////////////////////////////////////
/// \brief The process-wide token → engine-factory registry, plus its guard.
/// Function-local statics so there's no static-init-order dependency between
/// this translation unit and whoever calls RegisterWaveEngineFactory.
std::map<std::string, WaveEngineFactory> &Registry()
{
  static std::map<std::string, WaveEngineFactory> registry;
  return registry;
}

//////////////////////////////////////////////////
std::mutex &RegistryMutex()
{
  static std::mutex m;
  return m;
}
}  // namespace

//////////////////////////////////////////////////
void RegisterWaveEngineFactory(const std::string &_token,
                               WaveEngineFactory _factory)
{
  const std::lock_guard<std::mutex> lock(RegistryMutex());
  Registry()[_token] = std::move(_factory);
}

//////////////////////////////////////////////////
std::shared_ptr<IWaveField> CreateWaveSimulation(
  const std::string &_algorithm,
  const WaveParameters &_params)
{
  // Copy the factory out under the lock, then build/configure unlocked so a
  // factory can't deadlock against the registry (and a slow build doesn't
  // serialize other registrations).
  WaveEngineFactory factory;
  {
    const std::lock_guard<std::mutex> lock(RegistryMutex());
    auto it = Registry().find(_algorithm);
    if (it != Registry().end())
      factory = it->second;
  }

  if (!factory)
  {
    gzerr << "[CreateWaveSimulation] no engine registered for '"
          << _algorithm << "'. A consumer that links the engine must call "
          << "RegisterWaveEngineFactory first — the system plugins do this "
          << "on the server, the water visual on the GUI." << '\n';
    return nullptr;
  }

  auto field = factory();
  if (!field)
  {
    gzerr << "[CreateWaveSimulation] factory for '" << _algorithm
          << "' returned null" << '\n';
    return nullptr;
  }
  field->SetParameters(_params);
  return field;
}

}  // namespace gz::sim::waves
