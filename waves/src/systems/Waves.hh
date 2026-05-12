/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#ifndef GZ_SIM_SYSTEMS_WAVES_HH_
#define GZ_SIM_SYSTEMS_WAVES_HH_

#include <memory>

#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>

namespace gz::sim::systems
{
  /// \brief The Waves system parses wave parameters from SDF, samples the
  /// configured spectrum, and writes the result to a `Wavefield` component on
  /// the world entity. Consumers (water visual, buoyancy, hydrodynamics, …)
  /// read from this single source of truth.
  ///
  /// In `PreUpdate` the component is periodically re-marked as changed for
  /// the first few seconds of simulation so SceneBroadcaster will keep
  /// re-broadcasting it until the GUI process has loaded the component type
  /// (the GUI loads our plugin libraries lazily, and a one-time replication
  /// at world load tends to arrive before our type is registered there).
  class Waves : public System,
                public ISystemConfigure,
                public ISystemPreUpdate
  {
    public: Waves();
    public: ~Waves() override;

    public: void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &_eventMgr) override;

    public: void PreUpdate(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}  // namespace gz::sim::systems

#endif  // GZ_SIM_SYSTEMS_WAVES_HH_
