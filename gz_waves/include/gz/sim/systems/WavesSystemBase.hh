/*
 * Copyright (C) 2026 Honu Robotics
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
  /// engine, writing the `Wavefield` component on the world entity, and
  /// throttling `Update` — and defers only the *choice* of engine to the
  /// concrete subclass.
  ///
  /// A subclass is a complete loadable System: it inherits the Configure /
  /// PreUpdate behaviour and supplies just two things: the engine's token (for
  /// the GUI to reconstruct the matching engine) and a factory that builds and
  /// configures that engine. Because the subclass links its engine directly,
  /// no runtime plugin loader is involved on the server side.
  ///
  /// Each engine token names a *whole synthesis technique* (a fixed bundle of
  /// inverse transform, kinematics, and statistics), not a single design axis:
  /// `gerstner` is the analytic, direct summation, deterministic engine and
  /// `fft` is the FFT synthesized, random spectrum engine. The names are
  /// conventional shorthand, not a transform versus kinematics contrast (the
  /// `fft` engine also applies Gerstner-style displacement).
  ///
  /// The `Wavefield` component is written once in `Configure` and re-marked as
  /// changed only when its recipe actually changes (a `set_parameters` service
  /// call). A late-joining GUI process does not rely on a re-broadcast window;
  /// it pulls the current recipe on demand once its component type is
  /// registered.
  ///
  /// SDF is parsed in `ParseSdf`: `<update_rate>` at plugin level and a
  /// `<wave>` block of parameters (backed by `waves::WaveParameters` in
  /// `gz_waves/include/gz/sim/waves/Wavefield.hh`). The base parses every known
  /// tag, but **each concrete engine system documents — and uses — its own
  /// parameter surface**: see FftWaves (`gz-sim-waves-fft-system`) and
  /// GerstnerWaves (`gz-sim-waves-gerstner-system`). The same tag names are also
  /// accepted by the `/world/<name>/wave/set_parameters` service
  /// (`gz.msgs.Param`) for live changes.
  class WavesSystemBase : public System,
                          public ISystemConfigure,
                          public ISystemPreUpdate,
                          public ISystemReset
  {
    /// \brief Constructor.
    public: WavesSystemBase();
    /// \brief Destructor.
    public: ~WavesSystemBase() override;

    // Documentation inherited
    public: void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

    /// \brief On reset, rewind the update throttle so the wave field advances
    /// from t = 0 again. Sim time rewinds on a reset but the cached timestamps
    /// would not, which keeps the throttle false until time catches back up —
    /// freezing the field (and anything riding it). Reset rewinds *time* only:
    /// any parameters changed at runtime via `set_parameters` are intentionally
    /// retained (a reset does not restore the original SDF recipe).
    public: void Reset(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

    /// \brief Token recorded in the `Wavefield` component (e.g. "fft",
    /// "gerstner") so the GUI can reconstruct the matching engine on
    /// deserialization.
    /// \return The engine's registration token.
    protected: virtual std::string EngineToken() const = 0;

    /// \brief Build and fully configure (i.e. call `SetParameters`) the engine
    /// for this system from `_params`. Runs once, on the server, in Configure.
    /// \param[in] _params Wave parameters to build and configure the engine with.
    /// \return The configured engine instance.
    protected: virtual std::shared_ptr<waves::IWaveField> MakeEngine(
      const waves::WaveParameters &_params) const = 0;

    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}  // namespace gz::sim::systems

#endif  // GZ_SIM_SYSTEMS_WAVESSYSTEMBASE_HH_
