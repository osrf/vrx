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
#include <string>
#include <unordered_set>

#include <gz/common/SystemPaths.hh>
#include <gz/plugin/Loader.hh>

#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::waves
{

namespace
{
/// \brief Map a wave-field provider token to its plugin library base name and
/// registered class name, following the convention
/// "gz-waves-provider-<token>". Returns false for an unknown token.
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
}  // namespace

std::shared_ptr<IWaveField> CreateWaveSimulation(
  const std::string &_algorithm,
  const WaveParameters &_params)
{
  std::string libName, className;
  if (!ProviderForToken(_algorithm, libName, className))
  {
    std::cerr << "[CreateWaveSimulation] unknown provider '" << _algorithm
              << "'; supported: 'gerstner', 'fft'" << std::endl;
    return nullptr;
  }

  // Resolve the provider library on the plugin / shared-library search paths.
  gz::common::SystemPaths systemPaths;
  systemPaths.SetPluginPathEnv("GZ_SIM_SYSTEM_PLUGIN_PATH");
  if (const char *ldPath = std::getenv("LD_LIBRARY_PATH"))
    systemPaths.AddPluginPaths(ldPath);
  std::string pathToLib = systemPaths.FindSharedLibrary(libName);
  if (pathToLib.empty())
    pathToLib = libName;  // fall back to the bare soname (loader/dlopen search)

  // Load the provider library and instantiate the registered class. The
  // shared_ptr returned by QueryInterfaceSharedPtr keeps both the plugin
  // instance and its library alive, so this local Loader may safely go out of
  // scope when we return.
  gz::plugin::Loader loader;
  const std::unordered_set<std::string> plugins = loader.LoadLib(pathToLib);
  if (plugins.count(className) == 0)
  {
    std::cerr << "[CreateWaveSimulation] provider class '" << className
              << "' not found in '" << pathToLib << "' (provider '"
              << _algorithm << "')" << std::endl;
    return nullptr;
  }

  auto plugin = loader.Instantiate(className);
  if (!plugin)
  {
    std::cerr << "[CreateWaveSimulation] failed to instantiate '" << className
              << "'" << std::endl;
    return nullptr;
  }

  auto field = plugin->QueryInterfaceSharedPtr<IWaveField>();
  if (!field)
  {
    std::cerr << "[CreateWaveSimulation] '" << className
              << "' does not provide the IWaveField interface" << std::endl;
    return nullptr;
  }

  field->SetParameters(_params);
  return field;
}

}  // namespace gz::sim::waves
