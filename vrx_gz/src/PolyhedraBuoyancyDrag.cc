/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <mutex>
#include <string>
#include <ostream>
#include <vector>
#include <gz/common/Profiler.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include "gz/transport/Node.hh"
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>

#include "PolyhedraBuoyancyDrag.hh"
#include "ShapeVolume.hh"
#include "Wavefield.hh"

using namespace gz;
using namespace vrx;

/// \brief A class for storing buoyancy object properties.
///
/// ## Required parameters
///
/// * `<link_name>` is the name of the link used to apply forces.
/// * `<geometry>` is the geometry element modeling the buoyancy of the object.
///
/// ## Optional parameters
///
/// * `<pose>` is the offset from `link_name` where forces will be applied.
class BuoyancyObject
{
  /// \brief Load a buoyancy object from SDF.
  /// \param[in] _entity The model entity contining the buoyancy object.
  /// \param[in] _sdf The SDF Element tree containing the object parameters.
  /// \param[in] _ecm Gazebo's ECM.
  public: void Load(const sim::Entity &_entity,
                    const std::shared_ptr<const sdf::Element> &_sdf,
                    sim::EntityComponentManager &_ecm);

  /// \brief Stream extraction operator.
  /// \param[in] _out output stream.
  /// \param[in] _obj BuoyancyObject to output.
  /// \return The stream.
  public: friend std::ostream
      &operator<<(std::ostream &_out, const BuoyancyObject &_obj)
  {
    _out << "Buoyancy object:\n"
         << "\tLink: " << _obj.linkName << "[" << _obj.linkId << "]\n"
         << "\tPose: " << _obj.pose << std::endl
         << "\tGeometry: " << _obj.shape->Display() << std::endl
         << "\tMass: " << _obj.mass << std::endl
         << "\tWater height: " << _obj.height << std::endl
         << "\tWater velocity: " << _obj.waterSpeed << std::endl;

    return _out;
  }

  /// \brief The link entity.
  public: sim::Link link;

  /// \brief Associated link ID.
  public: sim::Entity linkId = -1;

  /// \brief Associated link name.
  public: std::string linkName = "";

  /// \brief Pose of buoyancy relative to link origin.
  public: math::Pose3d pose = math::Pose3d::Zero;

  /// \brief Object mass (from inertial elem).
  public: double mass = 0;

  /// \brief Buoyancy object's shape properties.
  public: ShapeVolumePtr shape = nullptr;

  /// \brief Water height at pose.
  public: double height = 0;

  /// \brief Water speed at pose.
  public: double waterSpeed = 0;
};

///////////////////////////////////////////////////
void BuoyancyObject::Load(const sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  sim::EntityComponentManager &_ecm)
{
  // Parse required <link_name>.
  if (!_sdf->HasElement("link_name"))
  {
    gzerr << "No <link_name> specified" << std::endl;
    return;
  }

  this->linkName = _sdf->Get<std::string>("link_name");
  sim::Model model(_entity);
  this->link = sim::Link(model.LinkByName(_ecm, this->linkName));
  if (!this->link.Valid(_ecm))
  {
    gzerr << "Could not find link named [" << this->linkName
          << "] in model" << std::endl;
    return;
  }
  this->link.EnableVelocityChecks(_ecm, true);
  this->linkId = this->link.Entity();

  // Get mass.
  auto inertial = _ecm.Component<sim::components::Inertial>(this->linkId);
  if (!inertial)
  {
    gzerr << "Could not inertial element for [" << this->linkName
          << "] in model" << std::endl;
    return;
  }
  this->mass = inertial->Data().MassMatrix().Mass();

  // Parse required <geometry>.
  if (!_sdf->HasElement("geometry"))
  {
    gzerr << "No <geometry> specified" << std::endl;
    return;
  }

  auto ptr = const_cast<sdf::Element *>(_sdf.get());
  sdf::ElementPtr geometry = ptr->GetElement("geometry");
  this->shape = std::move(ShapeVolume::makeShape(geometry));

  // Parse optional <pose>.
  if (_sdf->HasElement("pose"))
  {
    this->pose = _sdf->Get<math::Pose3d>("pose");
  }
}

/// \brief Private PolyhedraBuoyancyDrag data class.
class PolyhedraBuoyancyDrag::Implementation
{
  /// \brief Callback for receiving wave field updates.
  /// \param[in] _msg The message containing all the wave field parameters.
  public: void OnWavefield(const gz::msgs::Param &_msg);

  /// \brief The wavefield.
  public: Wavefield wavefield;

  /// \brief The density of the fluid in which the object is submerged in kg/m^3
  public: double fluidDensity = 997;

  /// \brief The height of the fluid/air interface [m]. Defaults to 0.
  public: double fluidLevel = 0;

  /// \brief Linear drag coefficient. Defaults to 0.
  public: double linearDrag = 0;

  /// \brief Angular drag coefficient. Defaults to 0.
  public: double angularDrag = 0;

  /// \brief List of buoyancy objects for model.
  public: std::vector<BuoyancyObject> buoyancyObjects;

  /// \brief Previous update time.
  public: double lastSimTime{0};

  /// \brief The world's gravity [m/s^2].
  public: math::Vector3d gravity;

  /// \brief A node for receiving wavefield updates.
  public: transport::Node node;

  /// \brief Mutex to protect the wavefield.
  public: std::mutex mutex;
};

//////////////////////////////////////////////////
void PolyhedraBuoyancyDrag::Implementation::OnWavefield(const msgs::Param &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->wavefield.Load(_msg);
}

//////////////////////////////////////////////////
PolyhedraBuoyancyDrag::PolyhedraBuoyancyDrag()
  : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void PolyhedraBuoyancyDrag::Configure(const sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    sim::EntityComponentManager &_ecm,
    sim::EventManager &/*_eventMgr*/)
{
  // Parse optional <wavefield>.
  this->dataPtr->wavefield.Load(_sdf);

  // Parse optional <fluid_density>.
  if (_sdf->HasElement("fluid_density"))
    this->dataPtr->fluidDensity = _sdf->Get<double>("fluid_density");

  // Parse optional <fluid_level>.
  if (_sdf->HasElement("fluid_level"))
    this->dataPtr->fluidLevel = _sdf->Get<double>("fluid_level");

  // Parse optional <linear_drag>.
  if (_sdf->HasElement("linear_drag"))
    this->dataPtr->linearDrag = _sdf->Get<double>("linear_drag");

  // Parse optional <angular_drag>.
  if (_sdf->HasElement("angular_drag"))
    this->dataPtr->angularDrag = _sdf->Get<double>("angular_drag");

  // Parse optional <buoyancy>.
  if (_sdf->HasElement("buoyancy"))
  {
    sim::Model model(_entity);
    gzdbg << "Found buoyancy element(s), looking at each element..."
          << std::endl;
    auto ptr = const_cast<sdf::Element *>(_sdf.get());
    for (sdf::ElementPtr buoyancyElem = ptr->GetElement("buoyancy");
        buoyancyElem;
        buoyancyElem = buoyancyElem->GetNextElement("buoyancy"))
    {
      BuoyancyObject buoyObj;
      buoyObj.Load(_entity, buoyancyElem, _ecm);
      buoyObj.height = this->dataPtr->fluidLevel;

      // Add buoyancy object to list and display stats.
      gzdbg << buoyObj << std::endl;
      this->dataPtr->buoyancyObjects.push_back(std::move(buoyObj));
    }
  }

  // Get the gravity from the world.
  auto worldEntity = sim::worldEntity(_ecm);
  auto world = sim::World(worldEntity);
  auto gravityOpt = world.Gravity(_ecm);
  if (!gravityOpt)
  {
    gzerr << "Unable to get the gravity from the world" << std::endl;
    return;
  }
  this->dataPtr->gravity = *gravityOpt;

  // Subscribe to receive wafefield parameters.
  this->dataPtr->node.Subscribe(this->dataPtr->wavefield.Topic(),
    &PolyhedraBuoyancyDrag::Implementation::OnWavefield, this->dataPtr.get());
}

//////////////////////////////////////////////////
void PolyhedraBuoyancyDrag::PreUpdate(const sim::UpdateInfo &_info,
    sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("PolyhedraBuoyancyDrag::PreUpdate");

  if (_info.paused)
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Elapsed time since the last update.
  double dt;
  double simTime;
  if (this->dataPtr->wavefield.Active())
  {
    simTime = std::chrono::duration<double>(_info.simTime).count();
    dt = simTime - this->dataPtr->lastSimTime;
    this->dataPtr->lastSimTime = simTime;
  }

  for (auto &buoyancyObj : this->dataPtr->buoyancyObjects)
  {
    // Link pose in world coordinates.
    auto worldPoseComp = buoyancyObj.link.WorldPose(_ecm);
    if (!worldPoseComp)
    {
      gzerr << "No worldpose component found" << std::endl;
      continue;
    }
    math::Pose3d linkFrame = *worldPoseComp;

    // Apply the offset.
    linkFrame = linkFrame * buoyancyObj.pose;

    // std::cerr << simTime << " " << linkFrame.Pos() << std::endl;

    if (this->dataPtr->wavefield.Active())
    {
      // Compute the wave displacement at the offset point,
      // relative to the mean water level.
      double waveHeight = this->dataPtr->wavefield.ComputeDepthSimply(
        linkFrame.Pos(), simTime);
      buoyancyObj.waterSpeed = (waveHeight - buoyancyObj.height) / dt;
      buoyancyObj.height = waveHeight;
    }

    auto submergedVolume = buoyancyObj.shape->CalculateVolume(linkFrame,
      buoyancyObj.height + this->dataPtr->fluidLevel);

    // Calculate buoyancy and drag forces.
    if (submergedVolume.volume > 1e-6)
    {
      // By Archimedes' principle,
      math::Vector3d buoyancy = -this->dataPtr->fluidDensity *
        submergedVolume.volume * this->dataPtr->gravity;

      float partialMass = buoyancyObj.mass * submergedVolume.volume
        / buoyancyObj.shape->volume;

      auto linVel = buoyancyObj.link.WorldLinearVelocity(_ecm);
      if (!linVel)
      {
        gzerr << "No linear velocity component found" << std::endl;
        continue;
      }

      // Linear drag.
      math::Vector3d relVel =
        math::Vector3d(0, 0, buoyancyObj.waterSpeed) - *linVel;

      math::Vector3d dragForce =
        this->dataPtr->linearDrag * partialMass * relVel;

      buoyancy += dragForce;

      if (buoyancy.Z() < 0.0)
        buoyancy.Z() = 0.0;

      // We apply a force with an offset. The offset should be in the center of
      // mass (COM) link's frame.
      auto comPose = buoyancyObj.link.WorldInertialPose(_ecm);
      auto centroidLocal = comPose->Rot().Inverse() *
        (submergedVolume.centroid - comPose->Pos());

      // Apply force.
      buoyancyObj.link.AddWorldForce(_ecm, buoyancy, centroidLocal);

      auto worldAngularVel = buoyancyObj.link.WorldAngularVelocity(_ecm);
      if (!worldAngularVel)
      {
        gzerr << "No angular velocity component found" <<"\n";
        continue;
      }

      auto localAngularVel = comPose->Rot().Inverse() * (*worldAngularVel);

      // Drag torque.
      double averageLength2 = ::pow(buoyancyObj.shape->averageLength, 2);
      math::Vector3d dragTorque = (-partialMass * this->dataPtr->angularDrag *
        averageLength2) * localAngularVel;

      math::Vector3d torqueWorld = (*comPose).Rot().RotateVector(dragTorque);

      // Apply torque.
      buoyancyObj.link.AddWorldWrench(_ecm, {0, 0, 0}, torqueWorld);
    }
  }
}

GZ_ADD_PLUGIN(PolyhedraBuoyancyDrag,
              sim::System,
              PolyhedraBuoyancyDrag::ISystemConfigure,
              PolyhedraBuoyancyDrag::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::PolyhedraBuoyancyDrag,
                    "vrx::PolyhedraBuoyancyDrag")
