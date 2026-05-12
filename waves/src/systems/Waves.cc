/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include "Waves.hh"

#include <chrono>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>

#include <sdf/Element.hh>

#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/waves/WaveSpectrum.hh"
#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::systems
{

namespace
{
// Re-mark the Wavefield component as changed for this long after Configure.
// The GUI process loads our plugin libraries (and therefore registers the
// component type) lazily, often after the initial scene state has arrived.
// Repeatedly marking the component as changed lets SceneBroadcaster keep
// re-sending it until the GUI side is ready to deserialize.
constexpr double kInitialReplicationSeconds = 5.0;
}

class Waves::Implementation
{
  public: void ParseSdf(const sdf::ElementPtr &_sdf);

  public: waves::WavefieldData data;
  public: Entity worldEnt{kNullEntity};
  public: ComponentTypeId componentType{0};
  public: bool componentReady{false};
  public: std::chrono::steady_clock::duration configureSimTime{0};
};

void Waves::Implementation::ParseSdf(const sdf::ElementPtr &_sdf)
{
  if (!_sdf->HasElement("wave"))
  {
    gzwarn << "[Waves] no <wave> element found; using defaults" << std::endl;
    return;
  }

  const auto wave = _sdf->GetElement("wave");
  auto &p = this->data.params;
  p.model     = wave->Get<std::string>("model",     p.model    ).first;
  p.number    = wave->Get<unsigned int>("number",   p.number   ).first;
  p.period    = wave->Get<double>("period",         p.period   ).first;
  p.amplitude = wave->Get<double>("amplitude",      p.amplitude).first;
  p.direction = wave->Get<double>("direction",      p.direction).first;
  p.angle     = wave->Get<double>("angle",          p.angle    ).first;
  p.scale     = wave->Get<double>("scale",          p.scale    ).first;
  p.steepness = wave->Get<double>("steepness",      p.steepness).first;
  p.phase     = wave->Get<double>("phase",          p.phase    ).first;
  p.tau       = wave->Get<double>("tau",            p.tau      ).first;
  p.gain      = wave->Get<double>("gain",           p.gain     ).first;
}

Waves::Waves() : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

Waves::~Waves() = default;

void Waves::Configure(
  const Entity &/*_entity*/,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm,
  EventManager &/*_eventMgr*/)
{
  this->dataPtr->ParseSdf(
    std::const_pointer_cast<sdf::Element>(_sdf));

  waves::SampleSpectrum(this->dataPtr->data);

  this->dataPtr->worldEnt = worldEntity(_ecm);
  if (this->dataPtr->worldEnt == kNullEntity)
  {
    gzerr << "[Waves] no world entity found; aborting" << std::endl;
    return;
  }

  _ecm.CreateComponent(this->dataPtr->worldEnt,
    components::Wavefield(this->dataPtr->data));
  this->dataPtr->componentType = components::Wavefield::typeId;
  this->dataPtr->componentReady = true;

  gzmsg << "[Waves] wavefield component created on world entity "
        << this->dataPtr->worldEnt
        << " (model=" << this->dataPtr->data.params.model
        << ", N=" << this->dataPtr->data.amplitudes.size()
        << ", generation=" << this->dataPtr->data.generation << ")"
        << std::endl;
}

void Waves::PreUpdate(
  const UpdateInfo &_info,
  EntityComponentManager &_ecm)
{
  if (!this->dataPtr->componentReady)
    return;

  if (this->dataPtr->configureSimTime ==
      std::chrono::steady_clock::duration{0})
  {
    this->dataPtr->configureSimTime = _info.simTime;
  }

  const double elapsed = std::chrono::duration<double>(
    _info.simTime - this->dataPtr->configureSimTime).count();

  if (elapsed > kInitialReplicationSeconds)
  {
    this->dataPtr->componentReady = false;
    return;
  }

  // Force SceneBroadcaster to re-broadcast the component to any process
  // (most importantly the GUI) that wasn't ready to deserialize it the
  // first time around.
  _ecm.SetChanged(this->dataPtr->worldEnt,
                  this->dataPtr->componentType,
                  ComponentState::OneTimeChange);
}

}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::Waves,
              gz::sim::System,
              gz::sim::systems::Waves::ISystemConfigure,
              gz::sim::systems::Waves::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::Waves, "gz::sim::systems::Waves")
