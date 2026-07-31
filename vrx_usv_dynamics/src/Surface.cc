/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <chrono>
#include <string>
#include <vector>
#include <gz/common/Profiler.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <sdf/Element.hh>

#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/waves/Eval.hh"
#include "gz/sim/World.hh"

#include "Surface.hh"

using namespace gz;
using namespace vrx;

/// \brief Private Surface data class.
class vrx::Surface::Implementation
{
  /// \brief Parse the points via SDF.
  /// \param[in] _sdf Pointer to the SDF.
  public: void ParsePoints(const std::shared_ptr<const sdf::Element> &_sdf);

  /// \brief The link entity.
  public: sim::Link link{sim::kNullEntity};

  /// \brief The world entity, which carries the Wavefield component.
  public: sim::Entity worldEntity{sim::kNullEntity};

  /// \brief Vessel length [m].
  public: double hullLength = 4.9;

  /// \brief Demi-hull radius [m].
  public: double hullRadius = 0.213;

  /// \brief Fluid height [m].
  public: double fluidLevel = 0;

  /// \brief Fluid density [kg/m^3].
  public: double fluidDensity = 1000.0;

  /// \brief The world's gravity [m/s^2].
  public: math::Vector3d gravity;

  /// \brief The points where the plugin applies forces. These points are
  /// relative to the link paramter's origin. Note that we don't check that the
  /// points are contained within the hull. You should pass reasonable points.
  public: std::vector<math::Vector3d> points;
};

//////////////////////////////////////////////////
void Surface::Implementation::ParsePoints(
  const std::shared_ptr<const sdf::Element> &_sdf)
{
  if (!_sdf->HasElement("points"))
    return;

  auto ptr = const_cast<sdf::Element *>(_sdf.get());
  auto sdfPoints = ptr->GetElement("points");

  // We need at least one point.
  if (!sdfPoints->HasElement("point"))
    gzerr << "Unable to find <points><point> element in SDF." << std::endl;

  auto pointElem = sdfPoints->GetElement("point");

  // Parse a new point.
  while (pointElem)
  {
    math::Vector3d point;
    pointElem->GetValue()->Get<math::Vector3d>(point);
    this->points.push_back(point);

    // Parse the next point.
    pointElem = pointElem->GetNextElement("point");
  }
}

//////////////////////////////////////////////////
Surface::Surface()
  : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void Surface::Configure(const sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    sim::EntityComponentManager &_ecm,
    sim::EventManager &/*_eventMgr*/)
{
  // Parse required elements.
  if (!_sdf->HasElement("link_name"))
  {
    gzerr << "No <link_name> specified" << std::endl;
    return;
  }

  sim::Model model(_entity);
  std::string linkName = _sdf->Get<std::string>("link_name");
  this->dataPtr->link = sim::Link(model.LinkByName(_ecm, linkName));
  if (!this->dataPtr->link.Valid(_ecm))
  {
    gzerr << "Could not find link named [" << linkName
           << "] in model" << std::endl;
    return;
  }

  // Optional parameters.
  // Although some of these parameters are required in this plugin, a potential
  // derived plugin might not need them. Make sure that the default values are
  // reasonable.
  if (_sdf->HasElement("hull_length"))
  {
    this->dataPtr->hullLength = _sdf->Get<double>("hull_length");
  }

  if (_sdf->HasElement("hull_radius"))
  {
    this->dataPtr->hullRadius = _sdf->Get<double>("hull_radius");
  }

  if (_sdf->HasElement("fluid_level"))
  {
    this->dataPtr->fluidLevel = _sdf->Get<double>("fluid_level");
  }

  if (_sdf->HasElement("fluid_density"))
  {
    this->dataPtr->fluidDensity = _sdf->Get<double>("fluid_density");
  }

  // Parse the optional <points> element.
  this->dataPtr->ParsePoints(_sdf);

  // Cache the world entity: it carries both the gravity and the Wavefield
  // component that PreUpdate samples every tick.
  this->dataPtr->worldEntity = sim::worldEntity(_ecm);
  if (this->dataPtr->worldEntity == sim::kNullEntity)
  {
    gzerr << "Unable to find the world entity" << std::endl;
    return;
  }

  // Get the gravity from the world.
  auto world = sim::World(this->dataPtr->worldEntity);
  auto gravityOpt = world.Gravity(_ecm);
  if (!gravityOpt)
  {
    gzerr << "Unable to get the gravity from the world" << std::endl;
    return;
  }
  this->dataPtr->gravity = *gravityOpt;

  // VRX 3 carried the wave field per-plugin, loaded from a <wavefield> block
  // and kept in sync over a transport topic. The wave field is now world state:
  // a single components::Wavefield on the world entity, written by the world's
  // wave source system. Warn rather than fail, for models we don't own.
  if (_sdf->HasElement("wavefield"))
  {
    gzwarn << "<wavefield> is ignored. The wave field now comes from the "
           << "Wavefield component on the world entity, written by the world's "
           << "wave source system (gz-sim-waves-*-system). Remove the "
           << "<wavefield> block from this plugin." << std::endl;
  }

  gzdbg << "Surface plugin successfully configured with the following "
         << "parameters:" << std::endl;
  gzdbg << "  <link_name>: " << linkName << std::endl;
  gzdbg << "  <hull_length>: " << this->dataPtr->hullLength << std::endl;
  gzdbg << "  <hull_radius>: " << this->dataPtr->hullRadius << std::endl;
  gzdbg << "  <fluid_level>: " << this->dataPtr->fluidLevel << std::endl;
  gzdbg << "  <fluid_density>: " << this->dataPtr->fluidDensity << std::endl;
  gzdbg << "  <points>:" << std::endl;
  for (const auto &p : this->dataPtr->points)
    gzdbg << "    [" << p << "]" << std::endl;
}

//////////////////////////////////////////////////
void Surface::PreUpdate(const sim::UpdateInfo &_info,
    sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("Surface::PreUpdate");

  if (_info.paused)
    return;

  // Vehicle frame transform.
  const auto kPose = this->dataPtr->link.WorldPose(_ecm);
  if (!kPose)
  {
    sim::enableComponent<sim::components::WorldPose>(
      _ecm, this->dataPtr->link.Entity(), true);

    gzerr << "Unable to get world pose from link ["
           << this->dataPtr->link.Entity() << "]" << std::endl;
    return;
  }
  const math::Vector3d kEuler = (*kPose).Rot().Euler();
  math::Quaternion vq(kEuler.X(), kEuler.Y(), kEuler.Z());

  const double kSimTime = std::chrono::duration<double>(_info.simTime).count();

  // The wave field is world state. Look it up per tick rather than caching it
  // in Configure: plugin load order between a world-level system and an
  // <include>d model's systems is not guaranteed, and a reset or a
  // SceneBroadcaster round-trip can rewrite the component. This is an O(1) map
  // lookup, and the world's wave source system runs before model systems, so
  // it is already valid this tick.
  //
  // Deliberately NOT calling waves::Advance() here: the wave source system owns
  // advancing the field, throttled to its <update_rate>. With two Surface
  // instances per vessel, advancing from a consumer would re-run the field
  // several times per tick.
  //
  // Deliberately NOT watching WavefieldData::generation either: nothing derived
  // from the recipe is cached here, so a live /wave/set_parameters call takes
  // effect on the next tick for free. (WaterVisual must watch it because it
  // rebuilds a private engine; this plugin queries the shared one.)
  const auto *kWavefieldComp =
    _ecm.Component<sim::components::Wavefield>(this->dataPtr->worldEntity);

  for (auto const &bpnt : this->dataPtr->points)
  {
    // Transform from vessel to fluid/world frame.
    const math::Vector3d kBpntW = vq * bpnt;

    // Vertical location of boat grid point in world frame.
    const float kDdz = (*kPose).Pos().Z() + kBpntW.Z();

    // World location of grid point.
    math::Vector3d point;
    point.X() = (*kPose).Pos().X() + kBpntW.X();
    point.Y() = (*kPose).Pos().Y() + kBpntW.Y();

    // Surface elevation above the still water level at the grid point [m],
    // positive up. This replaces VRX 3's Wavefield::ComputeDepthSimply, which
    // despite its name returned the same quantity with the same sign
    // (eta = sum a_i cos(k_i.x - omega_i t), times a startup ramp) - so the
    // arithmetic below is unchanged. Eval::SurfaceElevation is null-safe and
    // returns still water when no engine is attached; the component check
    // covers a world with no wave source system at all.
    const double kElevation = kWavefieldComp
      ? sim::waves::SurfaceElevation(
          kWavefieldComp->Data(), point.X(), point.Y(), kSimTime)
      : 0.0;

    // Total z location of boat grid point relative to fluid surface.
    double deltaZ = (this->dataPtr->fluidLevel + kElevation) - kDdz;
    // Enforce only upward buoy force
    deltaZ = std::max(deltaZ, 0.0);
    deltaZ = std::min(deltaZ, this->dataPtr->hullRadius);

    const float kBuoyForce =
      this->CircleSegment(this->dataPtr->hullRadius, deltaZ) *
        this->dataPtr->hullLength / this->dataPtr->points.size() *
          -this->dataPtr->gravity.Z() * this->dataPtr->fluidDensity;

    // Apply force at the point.
    // Position is in the link frame and force is in world frame.
    this->dataPtr->link.AddWorldForce(_ecm,
      math::Vector3d(0, 0, kBuoyForce),
      bpnt);

    // Debug output:
    // gzdbg << bpnt.X() << "," << bpnt.Y() << "," << bpnt.Z() << std::endl;
    // gzdbg << "elevation: " << kElevation << std::endl;
    // gzdbg << "dz: " << dz << std::endl;
    // gzdbg << "kDdz: " << kDdz << std::endl;
    // gzdbg << "deltaZ: " << deltaZ << std::endl;
    // gzdbg << "hull radius: " << this->dataPtr->hullRadius << std::endl;
    // gzdbg << "hull length: " << this->dataPtr->hullLength << std::endl;
    // gzdbg << "gravity z: " << -this->dataPtr->gravity.Z() << std::endl;
    // gzdbg << "fluid density: " << this->dataPtr->fluidDensity << std::endl;
    // gzdbg << "Force: " << kBuoyForce << std::endl << std::endl;
  }
}

//////////////////////////////////////////////////
math::Vector3d Surface::Gravity() const
{
  return this->dataPtr->gravity;
}

//////////////////////////////////////////////////
double Surface::HullLength() const
{
  return this->dataPtr->hullLength;
}

//////////////////////////////////////////////////
double Surface::HullRadius() const
{
  return this->dataPtr->hullRadius;
}

//////////////////////////////////////////////////
double Surface::FluidDensity() const
{
  return this->dataPtr->fluidDensity;
}

//////////////////////////////////////////////////
double Surface::CircleSegment(double _r, double _h) const
{
  return _r * _r * acos((_r -_h) / _r ) -
    (_r - _h) * sqrt(2 * _r * _h - _h * _h);
}

GZ_ADD_PLUGIN(Surface,
              sim::System,
              Surface::ISystemConfigure,
              Surface::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::Surface,
                    "vrx::Surface")
