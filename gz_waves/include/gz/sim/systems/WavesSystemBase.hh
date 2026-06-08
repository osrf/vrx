/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef GZ_SIM_SYSTEMS_WAVESSYSTEMBASE_HH_
#define GZ_SIM_SYSTEMS_WAVESSYSTEMBASE_HH_

#include <memory>
#include <string>

#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>

namespace gz::sim::waves
{
  class IWaveField;
  struct WaveParameters;
}

namespace gz::sim::systems
{
  /// \brief Shared base for the per-engine wave systems
  /// (`gz-sim-waves-fft-system`, `gz-sim-waves-gerstner-system`). It owns all
  /// the ECM / source plumbing — parsing the `<wave>` block, building the
  /// engine, writing the `Wavefield` component on the world entity, throttling
  /// `Update`, and re-marking the component for GUI replication — and defers
  /// only the *choice* of engine to the concrete subclass.
  ///
  /// A subclass is a complete loadable System: it inherits the Configure /
  /// PreUpdate behaviour and supplies just two things — the engine's token (for
  /// the GUI to reconstruct the matching backend) and a factory that builds and
  /// configures that backend. Because the subclass links its engine directly,
  /// no runtime plugin loader is involved on the server side.
  ///
  /// In `PreUpdate` the component is periodically re-marked as changed for the
  /// first few seconds of simulation so SceneBroadcaster keeps re-broadcasting
  /// it until the GUI process has registered the component type (the GUI loads
  /// our plugin libraries lazily, and a one-time replication at world load
  /// tends to arrive before our type is registered there).
  class WavesSystemBase : public System,
                          public ISystemConfigure,
                          public ISystemPreUpdate
  {
    public: WavesSystemBase();
    public: ~WavesSystemBase() override;

    public: void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &_eventMgr) override;

    public: void PreUpdate(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

    /// \brief Token recorded in the `Wavefield` component ("fft", "gerstner")
    /// so the GUI can reconstruct the matching engine on deserialization.
    protected: virtual std::string EngineToken() const = 0;

    /// \brief Build and fully configure (i.e. call `SetParameters`) the engine
    /// for this system from `_params`. Runs once, on the server, in Configure.
    protected: virtual std::shared_ptr<waves::IWaveField> MakeEngine(
      const waves::WaveParameters &_params) const = 0;

    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}  // namespace gz::sim::systems

#endif  // GZ_SIM_SYSTEMS_WAVESSYSTEMBASE_HH_
