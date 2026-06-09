/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include "WaveBuoyancy.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

#include <sdf/Element.hh>

#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/waves/Eval.hh"
#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::systems
{

namespace
{

//////////////////////////////////////////////////
/// \brief Area of a circular segment in a circle of radius r, where h is the
/// chord height measured from the bottom of the circle. Valid for h in [0, 2r].
///   A(0)   = 0
///   A(r)   = πr²/2  (half-submerged)
///   A(2r)  = πr²    (fully submerged)
double CircleSegment(double r, double h)
{
  return r * r * std::acos((r - h) / r) -
         (r - h) * std::sqrt(2.0 * r * h - h * h);
}

}  // namespace

class WaveBuoyancy::Implementation
{
  public: Link link{kNullEntity};
  public: double hullLength{4.9};
  public: double hullRadius{0.213};
  public: double fluidLevel{0.0};
  public: double fluidDensity{1000.0};
  public: gz::math::Vector3d gravity{0.0, 0.0, -9.80665};
  public: std::vector<gz::math::Vector3d> points;

  /// \brief Last sim time at which we advanced the wave field. We advance the
  /// instance we hold ourselves (see PreUpdate) rather than relying on the
  /// Waves system, so buoyancy stays correct even when the component we read
  /// is a replication round-trip copy. Throttled to the wavefield update rate.
  public: double lastWaveUpdate{-1.0};

  public: void ParsePoints(const sdf::ElementPtr &_sdf);
};

//////////////////////////////////////////////////
void WaveBuoyancy::Implementation::ParsePoints(const sdf::ElementPtr &_sdf)
{
  if (!_sdf->HasElement("points"))
    return;
  auto sdfPoints = _sdf->GetElement("points");
  if (!sdfPoints->HasElement("point"))
  {
    gzerr << "[WaveBuoyancy] <points> has no <point> children" << '\n';
    return;
  }
  for (auto p = sdfPoints->GetElement("point"); p;
       p = p->GetNextElement("point"))
  {
    gz::math::Vector3d pt;
    p->GetValue()->Get<gz::math::Vector3d>(pt);
    this->points.push_back(pt);
  }
}

//////////////////////////////////////////////////
WaveBuoyancy::WaveBuoyancy()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

WaveBuoyancy::~WaveBuoyancy() = default;

//////////////////////////////////////////////////
void WaveBuoyancy::Configure(
  const Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm,
  EventManager &/*_eventMgr*/)
{
  auto sdf = std::const_pointer_cast<sdf::Element>(_sdf);

  if (!sdf->HasElement("link_name"))
  {
    gzerr << "[WaveBuoyancy] <link_name> is required" << '\n';
    return;
  }
  const auto linkName = sdf->Get<std::string>("link_name");
  const Model model(_entity);
  this->dataPtr->link = Link(model.LinkByName(_ecm, linkName));
  if (!this->dataPtr->link.Valid(_ecm))
  {
    gzerr << "[WaveBuoyancy] link '" << linkName << "' not found" << '\n';
    return;
  }

  this->dataPtr->hullLength =
    sdf->Get<double>("hull_length",   this->dataPtr->hullLength  ).first;
  this->dataPtr->hullRadius =
    sdf->Get<double>("hull_radius",   this->dataPtr->hullRadius  ).first;
  this->dataPtr->fluidLevel =
    sdf->Get<double>("fluid_level",   this->dataPtr->fluidLevel  ).first;
  this->dataPtr->fluidDensity =
    sdf->Get<double>("fluid_density", this->dataPtr->fluidDensity).first;
  this->dataPtr->ParsePoints(sdf);

  if (this->dataPtr->points.empty())
  {
    gzerr << "[WaveBuoyancy] no buoyancy <points> defined for link '"
          << linkName << "'" << '\n';
    return;
  }

  if (auto gravOpt = World(worldEntity(_ecm)).Gravity(_ecm); gravOpt)
    this->dataPtr->gravity = *gravOpt;

  // Make sure the link has WorldPose populated each tick.
  this->dataPtr->link.EnableVelocityChecks(_ecm, true);

  gzmsg << "[WaveBuoyancy] configured on link '" << linkName << "': "
        << this->dataPtr->points.size() << " sample points, "
        << "hull " << this->dataPtr->hullLength << "x"
        << this->dataPtr->hullRadius << " m" << '\n';
}

//////////////////////////////////////////////////
void WaveBuoyancy::PreUpdate(
  const UpdateInfo &_info,
  EntityComponentManager &_ecm)
{
  GZ_PROFILE("WaveBuoyancy::PreUpdate");

  if (_info.paused)
    return;
  if (!this->dataPtr->link.Valid(_ecm))
    return;
  if (this->dataPtr->points.empty())
    return;

  const Entity worldEnt = worldEntity(_ecm);
  auto *wfComp = (worldEnt != kNullEntity)
    ? _ecm.Component<components::Wavefield>(worldEnt)
    : nullptr;

  // No wavefield → buoyancy reverts to still-water at fluid_level.
  // Acceptable behavior: the link still floats; just no wave excitation.

  auto poseOpt = this->dataPtr->link.WorldPose(_ecm);
  if (!poseOpt)
    return;
  const auto &pose = *poseOpt;
  const double t = std::chrono::duration<double>(_info.simTime).count();

  // Advance the wave field to the current time on the instance we actually
  // hold, instead of assuming the Waves system advanced it. The Wavefield
  // component serializes only the recipe (params + seed), so a copy obtained
  // through a replication round-trip is a freshly-constructed simulation that
  // has only run Update(0) — i.e. an identically flat field. Advancing it here
  // makes the elevation query correct regardless of which instance we got.
  // (Gerstner's Update() is a no-op, so this is free for it; FFT/Encino are
  // grid-based and stateful.) Throttled to the wavefield's update rate, and
  // idempotent in the simulation, so cost stays bounded across consumers.
  if (wfComp && wfComp->Data().simulation)
  {
    const double rate = wfComp->Data().updateRate;
    const double period = rate > 0.0 ? 1.0 / rate : 0.0;
    if (t - this->dataPtr->lastWaveUpdate >= period)
    {
      wfComp->Data().simulation->Update(t);
      this->dataPtr->lastWaveUpdate = t;
    }
  }

  const double r = this->dataPtr->hullRadius;
  const double maxImmersion = 2.0 * r;
  const double sliceLen =
    this->dataPtr->hullLength /
    static_cast<double>(this->dataPtr->points.size());
  const double gravZ = -this->dataPtr->gravity.Z();

  for (const auto &localPt : this->dataPtr->points)
  {
    const auto worldOffset = pose.Rot() * localPt;
    const auto worldPt = pose.Pos() + worldOffset;

    const double eta = wfComp
      ? waves::SurfaceElevation(wfComp->Data(), worldPt.X(), worldPt.Y(), t)
      : 0.0;

    const double waterZ = this->dataPtr->fluidLevel + eta;
    double deltaZ = waterZ - worldPt.Z();
    // Fix from old VRX: clamp upper bound to the full diameter (2r), not r.
    deltaZ = std::clamp(deltaZ, 0.0, maxImmersion);
    if (deltaZ <= 0.0)
      continue;

    const double force =
      CircleSegment(r, deltaZ) * sliceLen *
      this->dataPtr->fluidDensity * gravZ;

    this->dataPtr->link.AddWorldForce(
      _ecm, gz::math::Vector3d(0.0, 0.0, force), localPt);
  }
}

}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::WaveBuoyancy,
              gz::sim::System,
              gz::sim::systems::WaveBuoyancy::ISystemConfigure,
              gz::sim::systems::WaveBuoyancy::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::WaveBuoyancy,
                    "gz::sim::systems::WaveBuoyancy")
