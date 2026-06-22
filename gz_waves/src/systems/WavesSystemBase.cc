/*
 * Copyright (C) 2026 Honu Robotics
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

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/waves/Eval.hh"  // Advance(): the field-query facade
#include "gz/sim/waves/Wavefield.hh"
#include "gz/sim/waves/WaveSimulation.hh"  // drives the engine (producer side)

namespace gz::sim::systems
{

namespace
{
//////////////////////////////////////////////////
// --- set_parameters service: gz.msgs.Any -> scalar -------------------------
/// \brief Read a numeric Any (DOUBLE / INT32 / BOOLEAN) as a double.
/// \param[in]  _v   The value to read.
/// \param[out] _out Set to the numeric value on success.
/// \return true on a numeric value; false on a non-numeric one (so the caller
/// can warn and skip rather than corrupt a field).
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

//////////////////////////////////////////////////
/// \brief Read an integer Any (INT32, or a DOUBLE truncated) as an int.
/// \param[in]  _v   The value to read.
/// \param[out] _out Set to the integer value on success.
/// \return true on a numeric value; false otherwise.
bool ReadInt(const gz::msgs::Any &_v, int &_out)
{
  switch (_v.type())
  {
    case gz::msgs::Any::INT32:  _out = _v.int_value();                      return true;
    case gz::msgs::Any::DOUBLE: _out = static_cast<int>(_v.double_value()); return true;
    default: return false;
  }
}

//////////////////////////////////////////////////
/// \brief Merge the recognised keys in `_req` onto a copy of `_p` (partial
/// update: absent keys keep their current value). The recognised keys and their
/// types come from GZ_WAVES_PARAM_TABLE — the same source `operator<<`/`>>` and
/// ParseSdf use — so there is no separate list to keep in sync.
/// \param[in]  _p       The base parameters to merge the update onto.
/// \param[in]  _req     The requested key → value updates.
/// \param[out] _matched Set true if at least one wave parameter was recognised.
/// \return The merged parameters.
waves::WaveParameters ApplyParam(waves::WaveParameters _p,
    const gz::msgs::Param &_req, bool &_matched)
{
  auto warnType = [](const std::string &_k)
  {
    gzwarn << "[Waves] set_parameters: key '" << _k
           << "' has a non-numeric value; ignored" << '\n';
  };

  for (const auto &kv : _req.params())
  {
    const std::string &k = kv.first;
    const gz::msgs::Any &v = kv.second;
    double d = 0.0;
    int i = 0;
    bool handled = false;
#define GZ_WAVES_AP_DBL(m) \
    { if (ReadDouble(v, d)) { _p.m = d; _matched = true; } else warnType(k); }
#define GZ_WAVES_AP_SIZE(m) \
    { if (ReadInt(v, i)) { _p.m = static_cast<std::size_t>(i); _matched = true; }\
      else warnType(k); }
#define GZ_WAVES_AP_U32(m) \
    { if (ReadInt(v, i)) { _p.m = static_cast<std::uint32_t>(i); _matched = true;}\
      else warnType(k); }
#define GZ_WAVES_AP_INT(m) \
    { if (ReadInt(v, i)) { _p.m = i; _matched = true; } else warnType(k); }
#define GZ_WAVES_AP_STR(m) \
    { if (v.type() == gz::msgs::Any::STRING) { _p.m = v.string_value(); \
        _matched = true; } else warnType(k); }
#define GZ_WAVES_AP_BOOL(m) \
    { if (ReadDouble(v, d)) { _p.m = (d != 0.0); _matched = true; } \
      else warnType(k); }
#define GZ_WAVES_AP(member, name, kind) \
    if (!handled && k == name) { handled = true; GZ_WAVES_AP_##kind(member) }
    GZ_WAVES_PARAM_TABLE(GZ_WAVES_AP)
#undef GZ_WAVES_AP
#undef GZ_WAVES_AP_DBL
#undef GZ_WAVES_AP_SIZE
#undef GZ_WAVES_AP_U32
#undef GZ_WAVES_AP_INT
#undef GZ_WAVES_AP_STR
#undef GZ_WAVES_AP_BOOL
    if (!handled)
    {
      gzwarn << "[Waves] set_parameters: unknown key '" << k << "' ignored"
             << '\n';
    }
  }
  return _p;
}
}

class WavesSystemBase::Implementation
{
  /// \brief Parse `<update_rate>` (plugin level) and the `<wave>` parameter
  /// block from the plugin SDF into `data.params`; missing tags keep defaults.
  /// \param[in] _sdf The plugin's SDF element.
  public: void ParseSdf(const sdf::ElementPtr &_sdf);

  /// \brief The wave-field recipe and live engine written to the component.
  public: waves::WavefieldData data;
  /// \brief The world entity the Wavefield component is attached to.
  public: Entity worldEnt{kNullEntity};
  /// \brief Cached Wavefield component type id (for SetChanged on update).
  public: ComponentTypeId componentType{0};

  /// \brief Throttle backend updates to at most this rate [Hz]. Default 30 Hz
  /// matches asv_wave_sim; cheap for analytic Gerstner, sets a sensible
  /// ceiling on FFT IFFT cost.
  public: double updateRate{30.0};

  /// \brief Last sim time at which `simulation->Update` was called.
  public: double lastUpdateTime{-1.0};

  /// \brief Drain a queued runtime parameter update (from the set_parameters
  /// service) on the ECM thread: reconfigure the engine, bump the generation,
  /// and re-broadcast the Wavefield component. No-op when nothing is queued.
  /// \param[in] _ecm The entity-component manager holding the component.
  public: void ApplyPendingParams(EntityComponentManager &_ecm);

  /// \brief set_parameters service handler. Runs on a gz-transport thread, so
  /// it only validates + queues the new parameters; PreUpdate applies them.
  /// \param[in]  _req The requested parameter updates (gz.msgs.Param map).
  /// \param[out] _rep Set to true if at least one key was recognised.
  /// \return true (the service call always completes; `_rep` carries the
  /// recognised/ignored result).
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

//////////////////////////////////////////////////
void WavesSystemBase::Implementation::ParseSdf(const sdf::ElementPtr &_sdf)
{
  // Note: the engine is fixed by the concrete system (its plugin identity), so
  // there is no <algorithm> tag to parse here.
  this->updateRate =
    _sdf->Get<double>("update_rate", this->updateRate).first;

  if (!_sdf->HasElement("wave"))
  {
    gzwarn << "[Waves] no <wave> element found; using defaults" << '\n';
    return;
  }

  const auto wave = _sdf->GetElement("wave");
  auto &p = this->data.params;
  // Every <wave> tag and its type comes from GZ_WAVES_PARAM_TABLE — the same
  // source serialization and the set_parameters service use. (Tags not relevant
  // to a given engine — e.g. the FFT/EncinoWaves selectors for Gerstner — are
  // simply ignored by that backend; parsing them here is harmless.)
#define GZ_WAVES_SDF_DBL(m, name)  p.m = wave->Get<double>(name, p.m).first;
#define GZ_WAVES_SDF_SIZE(m, name) \
    p.m = wave->Get<unsigned int>(name, static_cast<unsigned int>(p.m)).first;
#define GZ_WAVES_SDF_U32(m, name) \
    p.m = wave->Get<unsigned int>(name, static_cast<unsigned int>(p.m)).first;
#define GZ_WAVES_SDF_INT(m, name)  p.m = wave->Get<int>(name, p.m).first;
#define GZ_WAVES_SDF_STR(m, name)  p.m = wave->Get<std::string>(name, p.m).first;
#define GZ_WAVES_SDF_BOOL(m, name) p.m = wave->Get<bool>(name, p.m).first;
#define GZ_WAVES_SDF(member, name, kind) GZ_WAVES_SDF_##kind(member, name)
  GZ_WAVES_PARAM_TABLE(GZ_WAVES_SDF)
#undef GZ_WAVES_SDF
#undef GZ_WAVES_SDF_DBL
#undef GZ_WAVES_SDF_SIZE
#undef GZ_WAVES_SDF_U32
#undef GZ_WAVES_SDF_INT
#undef GZ_WAVES_SDF_STR
#undef GZ_WAVES_SDF_BOOL
}

//////////////////////////////////////////////////
WavesSystemBase::WavesSystemBase()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

WavesSystemBase::~WavesSystemBase() = default;

//////////////////////////////////////////////////
void WavesSystemBase::Configure(
  const Entity &/*_entity*/,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm,
  EventManager &/*_eventMgr*/)
{
  // ParseSdf only reads _sdf; the const_cast is solely to satisfy
  // sdf::Element::Get/GetElement, whose signatures are non-const.
  this->dataPtr->ParseSdf(
    std::const_pointer_cast<sdf::Element>(_sdf));

  this->dataPtr->worldEnt = worldEntity(_ecm);
  if (this->dataPtr->worldEnt == kNullEntity)
  {
    gzerr << "[Waves] no world entity found; aborting" << '\n';
    return;
  }

  // Drive the wave physics with the world's configured gravity so dispersion,
  // spectrum and sea-state period stay consistent with buoyancy and dynamics.
  if (auto gravOpt = World(this->dataPtr->worldEnt).Gravity(_ecm); gravOpt)
    this->dataPtr->data.params.gravity = gravOpt->Length();

  // The engine is determined by this concrete system, not by SDF.
  this->dataPtr->data.algorithm = this->EngineToken();
  this->dataPtr->data.simulation = this->MakeEngine(this->dataPtr->data.params);
  if (!this->dataPtr->data.simulation)
  {
    gzerr << "[Waves] failed to create '" << this->dataPtr->data.algorithm
          << "' engine; aborting" << '\n';
    return;
  }
  this->dataPtr->data.generation = 1;
  this->dataPtr->data.updateRate = this->dataPtr->updateRate;
  this->dataPtr->currentParams = this->dataPtr->data.params;

  _ecm.CreateComponent(this->dataPtr->worldEnt,
    components::Wavefield(this->dataPtr->data));
  this->dataPtr->componentType = components::Wavefield::typeId;

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
          << " (gz.msgs.Param -> gz.msgs.Boolean)" << '\n';
  }
  else
  {
    gzwarn << "[Waves] failed to advertise '" << service << "'" << '\n';
  }

  gzmsg << "[Waves] wavefield component created on world entity "
        << this->dataPtr->worldEnt
        << " (algorithm=" << this->dataPtr->data.algorithm
        << ", model=" << this->dataPtr->data.params.model
        << ", seaState=" << this->dataPtr->data.params.seaState
        << ", generation=" << this->dataPtr->data.generation << ")"
        << '\n';
}

//////////////////////////////////////////////////
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

  // The Wavefield component serializes recipe-only, so any ECM deserialize —
  // a reset's state restore or a SceneBroadcaster replication round-trip —
  // clears the component's live engine pointer (operator>>). Server-side
  // consumers (e.g. WaveBuoyancy) read the engine straight out of the
  // component, so re-point it at our authoritative, always-advanced instance
  // whenever the ECM has cleared it. Without this the buoy samples flat water
  // and stops responding to the waves after a reset. (World systems run before
  // model systems, so the buoy sees the restored pointer the same tick.)
  if (auto *comp =
        _ecm.Component<components::Wavefield>(this->dataPtr->worldEnt);
      comp && !comp->Data().simulation)
  {
    comp->Data().simulation = this->dataPtr->data.simulation;
  }

  // Throttle backend updates. Analytic Gerstner has a no-op Update(); FFT
  // regenerates the height grid each call (~ms at 128²). Skip entirely while
  // paused — the field is frozen, so there's nothing to advance, and consumers
  // (e.g. WaveBuoyancy) advance the field themselves, so correctness no longer
  // depends on this call landing in any particular tick.
  const double updatePeriod =
    this->dataPtr->updateRate > 0.0 ? 1.0 / this->dataPtr->updateRate : 0.0;
  if (!_info.paused &&
      simTime - this->dataPtr->lastUpdateTime >= updatePeriod)
  {
    waves::Advance(this->dataPtr->data, simTime);
    this->dataPtr->lastUpdateTime = simTime;
  }
}

//////////////////////////////////////////////////
void WavesSystemBase::Reset(
  const UpdateInfo & /*_info*/, EntityComponentManager & /*_ecm*/)
{
  // A reset rewinds sim time to 0. PreUpdate's throttle gates on
  // (simTime - lastUpdateTime >= period); with a stale lastUpdateTime it stays
  // false until sim time catches back up, so the engine never advances and the
  // field freezes. Rewind the throttle so the field advances from t = 0 again.
  this->dataPtr->lastUpdateTime = -1.0;
  gzmsg << "[Waves] reset: re-advancing the wave field from t=0" << '\n';
}

//////////////////////////////////////////////////
void WavesSystemBase::Implementation::ApplyPendingParams(
  EntityComponentManager &_ecm)
{
  waves::WaveParameters params;
  {
    const std::lock_guard<std::mutex> lock(this->paramMutex);
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
        << ", seaState=" << params.seaState << ")" << '\n';
}

//////////////////////////////////////////////////
bool WavesSystemBase::Implementation::OnSetParameters(
  const gz::msgs::Param &_req, gz::msgs::Boolean &_rep)
{
  bool matched = false;
  {
    const std::lock_guard<std::mutex> lock(this->paramMutex);
    const waves::WaveParameters base =
      this->pendingParams ? *this->pendingParams : this->currentParams;
    const waves::WaveParameters updated = ApplyParam(base, _req, matched);
    if (matched)
      this->pendingParams = updated;
  }
  if (!matched)
    gzwarn << "[Waves] set_parameters: no recognised wave parameter keys"
           << '\n';
  _rep.set_data(matched);
  return true;
}

}  // namespace gz::sim::systems
