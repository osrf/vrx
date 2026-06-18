/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef GZ_SIM_COMPONENTS_WAVEFIELD_HH_
#define GZ_SIM_COMPONENTS_WAVEFIELD_HH_

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  /// \brief A world-attached ECM component holding the current wave field
  /// state. Written by the wave source system; read by buoyancy,
  /// hydrodynamics, the water visual, and any wave-aware plugin.
  using Wavefield = Component<waves::WavefieldData, class WavefieldTag>;

  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.Wavefield", Wavefield)
}
}
}  // namespace gz::sim

#endif  // GZ_SIM_COMPONENTS_WAVEFIELD_HH_
