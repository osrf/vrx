/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include "gz/sim/systems/WavesSystemBase.hh"

#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Wavefield.hh"
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

// --- set_parameters service: gz.msgs.Any -> scalar -------------------------
// Read a numeric Any (DOUBLE / INT32 / BOOLEAN) as a double; false on a
// non-numeric value so the caller can warn and skip rather than corrupt a field.
bool ReadDouble(const gz::msgs::Any &_v, double &_out)
{
  switch (_v.type())
  {
    case gz::msgs::Any::DOUBLE:  _out = _v.double_value();           return true;
    case gz::msgs::Any::INT32:   _out = _v.int_value();              return true;
    case gz::msgs::Any::BOOLEAN: _out = _v.bool_value() ? 1.0 : 0.0; return true;
    default: return false;
  }
}

// Read an integer Any (INT32, or a DOUBLE truncated) as an int.
bool ReadInt(const gz::msgs::Any &_v, int &_out)
{
  switch (_v.type())
  {
    case gz::msgs::Any::INT32:  _out = _v.int_value();                      return true;
    case gz::msgs::Any::DOUBLE: _out = static_cast<int>(_v.double_value()); return true;
    default: return false;
  }
}

// Merge the recognised keys in `_req` onto a copy of `_base` (partial update:
// absent keys keep their current value). Keys mirror the <wave> SDF tags parsed
// in ParseSdf — keep the two lists in sync. Sets `_matched` if at least one
// wave parameter was recognised.
waves::WaveParameters ApplyParam(waves::WaveParameters _p,
    const gz::msgs::Param &_req, bool &_matched)
{
  // Double-valued tags (key -> destination field in the copy `_p`).
  const std::unordered_map<std::string, double *> doubles{
    {"period", &_p.period},     {"amplitude", &_p.amplitude},
    {"direction", &_p.direction}, {"angle", &_p.angle},
    {"scale", &_p.scale},       {"steepness", &_p.steepness},
    {"phase", &_p.phase},       {"tau", &_p.tau},
    {"gain", &_p.gain},         {"tile_size", &_p.tileSize},
    {"choppiness", &_p.choppiness}};

  // String-valued tags (key -> destination field).
  const std::unordered_map<std::string, std::string *> strings{
    {"model", &_p.model},         {"spectrum", &_p.spectrum},
    {"spreading", &_p.spreading}, {"dispersion", &_p.dispersion}};

  auto warnType = [](const std::string &_k)
  {
    gzwarn << "[Waves] set_parameters: key '" << _k
           << "' has a non-numeric value; ignored" << std::endl;
  };

  for (const auto &kv : _req.params())
  {
    const std::string &k = kv.first;
    const gz::msgs::Any &v = kv.second;
    double d = 0.0;
    int i = 0;
    if (auto it = doubles.find(k); it != doubles.end())
    {
      if (ReadDouble(v, d)) { *it->second = d; _matched = true; }
      else warnType(k);
    }
    else if (auto sit = strings.find(k); sit != strings.end())
    {
      if (v.type() == gz::msgs::Any::STRING)
      { *sit->second = v.string_value(); _matched = true; }
      else warnType(k);
    }
    else if (k == "number")
    { if (ReadInt(v, i)) { _p.number = static_cast<std::size_t>(i); _matched = true; } else warnType(k); }
    else if (k == "grid_size")
    { if (ReadInt(v, i)) { _p.gridSize = static_cast<std::size_t>(i); _matched = true; } else warnType(k); }
    else if (k == "seed")
    { if (ReadInt(v, i)) { _p.seed = static_cast<std::uint32_t>(i); _matched = true; } else warnType(k); }
    else if (k == "sea_state")
    { if (ReadInt(v, i)) { _p.seaState = i; _matched = true; } else warnType(k); }
    else
    {
      gzwarn << "[Waves] set_parameters: unknown key '" << k << "' ignored"
             << std::endl;
    }
  }
  return _p;
}
}

class WavesSystemBase::Implementation
{
  public: void ParseSdf(const sdf::ElementPtr &_sdf);

  public: waves::WavefieldData data;
  public: Entity worldEnt{kNullEntity};
  public: ComponentTypeId componentType{0};
  public: bool componentReady{false};
  public: std::chrono::steady_clock::duration configureSimTime{0};

  /// \brief Throttle backend updates to at most this rate [Hz]. Default 30 Hz
  /// matches asv_wave_sim; cheap for analytic Gerstner, sets a sensible
  /// ceiling on FFT IFFT cost.
  public: double updateRate{30.0};

  /// \brief Last sim time at which `simulation->Update` was called.
  public: double lastUpdateTime{-1.0};

  /// \brief Drain a queued runtime parameter update (from the set_parameters
  /// service) on the ECM thread: reconfigure the engine, bump the generation,
  /// and re-broadcast the Wavefield component. No-op when nothing is queued.
  public: void ApplyPendingParams(EntityComponentManager &_ecm);

  /// \brief set_parameters service handler. Runs on a gz-transport thread, so
  /// it only validates + queues the new parameters; PreUpdate applies them.
  public: bool OnSetParameters(
    const gz::msgs::Param &_req, gz::msgs::Boolean &_rep);

  /// \brief Transport node owning the set_parameters service.
  public: gz::transport::Node node;
  /// \brief Guards `currentParams` + `pendingParams` across the transport and
  /// ECM threads.
  public: std::mutex paramMutex;
  /// \brief Latest applied wave parameters — the base a partial service update
  /// is merged onto. Mutex-protected (the transport thread reads it).
  public: waves::WaveParameters currentParams;
  /// \brief Parameters queued by the service, applied in the next PreUpdate.
  public: std::optional<waves::WaveParameters> pendingParams;
};

void WavesSystemBase::Implementation::ParseSdf(const sdf::ElementPtr &_sdf)
{
  // Note: the engine is fixed by the concrete system (its plugin identity), so
  // there is no <algorithm> tag to parse here.
  this->updateRate =
    _sdf->Get<double>("update_rate", this->updateRate).first;

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

  // FFT-only parameters; ignored by Gerstner. Kept in <wave> for locality.
  p.tileSize   = wave->Get<double>("tile_size",       p.tileSize  ).first;
  p.gridSize   = wave->Get<unsigned int>("grid_size", p.gridSize  ).first;
  p.seed       = wave->Get<unsigned int>("seed",      p.seed      ).first;
  p.choppiness = wave->Get<double>("choppiness",      p.choppiness).first;

  // FFT spectrum selectors (EncinoWaves); ignored by the Gerstner engine.
  p.spectrum   = wave->Get<std::string>("spectrum",   p.spectrum  ).first;
  p.spreading  = wave->Get<std::string>("spreading",  p.spreading ).first;
  p.dispersion = wave->Get<std::string>("dispersion", p.dispersion).first;

  // High-level convenience: a WMO sea state code (0-9) that each engine turns
  // into a matching significant wave height + peak period (see WithSeaState).
  // When set, it overrides <period> and <gain>.
  p.seaState   = wave->Get<int>("sea_state",          p.seaState  ).first;
}

WavesSystemBase::WavesSystemBase()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

WavesSystemBase::~WavesSystemBase() = default;

void WavesSystemBase::Configure(
  const Entity &/*_entity*/,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm,
  EventManager &/*_eventMgr*/)
{
  this->dataPtr->ParseSdf(
    std::const_pointer_cast<sdf::Element>(_sdf));

  // The engine is determined by this concrete system, not by SDF.
  this->dataPtr->data.algorithm = this->EngineToken();
  this->dataPtr->data.simulation = this->MakeEngine(this->dataPtr->data.params);
  if (!this->dataPtr->data.simulation)
  {
    gzerr << "[Waves] failed to create '" << this->dataPtr->data.algorithm
          << "' engine; aborting" << std::endl;
    return;
  }
  this->dataPtr->data.generation = 1;
  this->dataPtr->data.updateRate = this->dataPtr->updateRate;
  this->dataPtr->currentParams = this->dataPtr->data.params;

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

  // Advertise a runtime parameter-update service. Callers send a gz.msgs.Param
  // map of <wave> tag names -> values (partial: omitted keys keep their current
  // value) and get a gz.msgs.Boolean ack. The change is queued here and applied
  // on the ECM thread in PreUpdate.
  std::string worldName;
  if (auto *nameComp =
        _ecm.Component<components::Name>(this->dataPtr->worldEnt))
    worldName = nameComp->Data();
  const std::string service =
    "/world/" + worldName + "/wave/set_parameters";
  if (this->dataPtr->node.Advertise(service,
        &Implementation::OnSetParameters, this->dataPtr.get()))
  {
    gzmsg << "[Waves] runtime parameter service: " << service
          << " (gz.msgs.Param -> gz.msgs.Boolean)" << std::endl;
  }
  else
  {
    gzwarn << "[Waves] failed to advertise '" << service << "'" << std::endl;
  }

  gzmsg << "[Waves] wavefield component created on world entity "
        << this->dataPtr->worldEnt
        << " (algorithm=" << this->dataPtr->data.algorithm
        << ", model=" << this->dataPtr->data.params.model
        << ", seaState=" << this->dataPtr->data.params.seaState
        << ", generation=" << this->dataPtr->data.generation << ")"
        << std::endl;
}

void WavesSystemBase::PreUpdate(
  const UpdateInfo &_info,
  EntityComponentManager &_ecm)
{
  if (!this->dataPtr->data.simulation)
    return;

  // Drain any runtime parameter update queued by the set_parameters service.
  this->dataPtr->ApplyPendingParams(_ecm);

  const double simTime = std::chrono::duration<double>(
    _info.simTime).count();

  // Throttle backend updates. Analytic Gerstner has a no-op Update(); FFT
  // regenerates the height grid each call (~ms at 128²). Skip entirely while
  // paused — the field is frozen, so there's nothing to advance, and consumers
  // (e.g. WaveBuoyancy) advance the field themselves, so correctness no longer
  // depends on this call landing in any particular tick. The replication
  // marking below still runs so the GUI can pick up the wavefield while paused.
  const double updatePeriod =
    this->dataPtr->updateRate > 0.0 ? 1.0 / this->dataPtr->updateRate : 0.0;
  if (!_info.paused &&
      simTime - this->dataPtr->lastUpdateTime >= updatePeriod)
  {
    this->dataPtr->data.simulation->Update(simTime);
    this->dataPtr->lastUpdateTime = simTime;
  }

  // SceneBroadcaster replication. Custom components don't reliably reach
  // the GUI on the initial scene state (it arrives before the GUI's plugin
  // libraries have registered our type). Re-marking the component as
  // changed for the first few seconds lets us catch the GUI when it's
  // ready.
  if (this->dataPtr->componentReady)
  {
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
    }
    else
    {
      _ecm.SetChanged(this->dataPtr->worldEnt,
                      this->dataPtr->componentType,
                      ComponentState::OneTimeChange);
    }
  }
}

void WavesSystemBase::Implementation::ApplyPendingParams(
  EntityComponentManager &_ecm)
{
  waves::WaveParameters params;
  {
    std::lock_guard<std::mutex> lock(this->paramMutex);
    if (!this->pendingParams)
      return;
    params = *this->pendingParams;
    this->pendingParams.reset();
    this->currentParams = params;
  }

  // Reconfigure the server-side engine in place. WaveBuoyancy reads this same
  // instance out of the component each tick, so it picks up the change with no
  // extra signalling; WaterVisual (GUI) rebuilds its own engine off the bumped
  // generation below. `data` is touched only on this (ECM) thread.
  this->data.params = params;
  this->data.simulation->SetParameters(params);
  this->data.generation += 1;
  this->lastUpdateTime = -1.0;  // advance the field next tick, unthrottled

  if (auto *comp = _ecm.Component<components::Wavefield>(this->worldEnt))
  {
    comp->Data() = this->data;
    _ecm.SetChanged(this->worldEnt, this->componentType,
                    ComponentState::OneTimeChange);
  }

  gzmsg << "[Waves] runtime parameters applied (generation="
        << this->data.generation << ", model=" << params.model
        << ", seaState=" << params.seaState << ")" << std::endl;
}

bool WavesSystemBase::Implementation::OnSetParameters(
  const gz::msgs::Param &_req, gz::msgs::Boolean &_rep)
{
  bool matched = false;
  {
    std::lock_guard<std::mutex> lock(this->paramMutex);
    const waves::WaveParameters base =
      this->pendingParams ? *this->pendingParams : this->currentParams;
    const waves::WaveParameters updated = ApplyParam(base, _req, matched);
    if (matched)
      this->pendingParams = updated;
  }
  if (!matched)
    gzwarn << "[Waves] set_parameters: no recognised wave parameter keys"
           << std::endl;
  _rep.set_data(matched);
  return true;
}

}  // namespace gz::sim::systems
