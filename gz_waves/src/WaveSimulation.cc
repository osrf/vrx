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

#include <cstdlib>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <unordered_set>

#include <gz/common/SystemPaths.hh>
#include <gz/plugin/Loader.hh>

#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::waves
{

namespace
{
/// \brief The process-wide token → engine-factory registry, plus its guard.
/// Function-local statics so there's no static-init-order dependency between
/// this translation unit and whoever calls RegisterWaveEngineFactory.
std::map<std::string, WaveEngineFactory> &Registry()
{
  static std::map<std::string, WaveEngineFactory> registry;
  return registry;
}

std::mutex &RegistryMutex()
{
  static std::mutex m;
  return m;
}

// ---------------------------------------------------------------------------
// TEMPORARY dlopen fallback (transition only).
//
// Used when a token isn't in the registry yet — e.g. during the migration to
// per-engine system plugins, before every consumer registers its engine. This
// whole block (and the gz-plugin / SystemPaths dependency) is removed once the
// registry is populated everywhere; see the provider-refactor plan.
// ---------------------------------------------------------------------------
bool ProviderForToken(const std::string &_token,
                      std::string &_libName,
                      std::string &_className)
{
  if (_token == "gerstner")
  {
    _libName   = "gz-waves-provider-gerstner";
    _className = "gz::sim::waves::GerstnerWaveSimulation";
    return true;
  }
  if (_token == "fft")
  {
    _libName   = "gz-waves-provider-fft";
    _className = "gz::sim::waves::FFTWaveSimulation";
    return true;
  }
  return false;
}

std::shared_ptr<IWaveField> TryLegacyDlopen(const std::string &_algorithm,
                                            const WaveParameters &_params)
{
  std::string libName, className;
  if (!ProviderForToken(_algorithm, libName, className))
    return nullptr;

  gz::common::SystemPaths systemPaths;
  systemPaths.SetPluginPathEnv("GZ_SIM_SYSTEM_PLUGIN_PATH");
  if (const char *ldPath = std::getenv("LD_LIBRARY_PATH"))
    systemPaths.AddPluginPaths(ldPath);
  std::string pathToLib = systemPaths.FindSharedLibrary(libName);
  if (pathToLib.empty())
    pathToLib = libName;

  gz::plugin::Loader loader;
  const std::unordered_set<std::string> plugins =
      loader.LoadLib(pathToLib, /*_noDelete=*/true);
  if (plugins.count(className) == 0)
    return nullptr;

  auto plugin = loader.Instantiate(className);
  if (!plugin)
    return nullptr;

  auto field = plugin->QueryInterfaceSharedPtr<IWaveField>();
  if (!field)
    return nullptr;

  field->SetParameters(_params);
  return field;
}
}  // namespace

void RegisterWaveEngineFactory(const std::string &_token,
                               WaveEngineFactory _factory)
{
  std::lock_guard<std::mutex> lock(RegistryMutex());
  Registry()[_token] = std::move(_factory);
}

std::shared_ptr<IWaveField> CreateWaveSimulation(
  const std::string &_algorithm,
  const WaveParameters &_params)
{
  // Copy the factory out under the lock, then build/configure unlocked so a
  // factory can't deadlock against the registry (and a slow build doesn't
  // serialize other registrations).
  WaveEngineFactory factory;
  {
    std::lock_guard<std::mutex> lock(RegistryMutex());
    auto it = Registry().find(_algorithm);
    if (it != Registry().end())
      factory = it->second;
  }

  if (factory)
  {
    auto field = factory();
    if (!field)
    {
      std::cerr << "[CreateWaveSimulation] factory for '" << _algorithm
                << "' returned null" << std::endl;
      return nullptr;
    }
    field->SetParameters(_params);
    return field;
  }

  // Not registered (yet) — fall back to the legacy dlopen loader.
  if (auto field = TryLegacyDlopen(_algorithm, _params))
    return field;

  std::cerr << "[CreateWaveSimulation] no engine registered for '"
            << _algorithm << "' and no provider library found; supported "
            << "tokens: 'gerstner', 'fft'" << std::endl;
  return nullptr;
}

}  // namespace gz::sim::waves
