/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <gz/msgs/pose.pb.h>
#include <memory>
#include <string>
#include <vector>
#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

#include "WildlifeScoringPlugin.hh"

using namespace gz;
using namespace vrx;

/// \brief All buoy goals.
enum class BuoyGoal
{
  /// \brief The goal is to stay out of the activation area.
  AVOID,

  /// \brief The goal is to circumnavigate the buoy clockwise.
  CIRCUMNAVIGATE_CLOCKWISE,

  /// \brief The goal is to circumnavigate the buoy counterclockwise
  CIRCUMNAVIGATE_COUNTERCLOCKWISE,
};

/// \brief All buoy states.
enum class BuoyState
{
  /// \brief Not "in" the gate and never engaged.
  NEVER_ENGAGED,

  /// \brief Not "in" the gate but was engaged at some point.
  NOT_ENGAGED,

  /// \brief Inside the area of activation of the gate.
  ENGAGED,

  /// \brief Succesfully circumnavigated.
  CIRCUMNAVIGATED,
};

/// \brief All gate states.
enum class GateState
{
  /// \brief Not "in" the gate.
  VEHICLE_OUTSIDE,

  /// \brief Before the gate.
  VEHICLE_BEFORE,

  /// \brief After the gate.
  VEHICLE_AFTER,

  /// \brief Gate crossed!
  CROSSED,
};

/// \brief A virtual gate to help detecting circumnavigation.
class VirtualGate
{
  /// \brief Constructor.
  /// \param[in] _leftMarkerEntity The Entity of the buoy.
  /// \param[in] _offset The offset from the buoy that delimitates the gate.
  /// \param[in] _width The width of the gate.
  /// \param[in out] _ecm The Entity Component Manager.
  public: VirtualGate(const sim::Entity _leftMarkerEntity,
                      const math::Vector3d &_offset,
                      double _width,
                      sim::EntityComponentManager &_ecm);

  /// \brief Where is the given robot pose with respect to the gate?
  /// \param _robotWorldPose Pose of the robot, in the world frame.
  /// \return The gate state given the current robot pose.
  public: GateState IsPoseInGate(const math::Pose3d &_robotWorldPose) const;

  /// \brief Recalculate the pose of the gate.
  public: void Update();

  /// \brief The left marker (buoy) entity.
  public: sim::Entity leftMarkerEntity = sim::kNullEntity;

  /// \brief The offset of the right marker with respect to the left marker
  /// in world coordinates.
  public: math::Vector3d offset;

  /// \brief The width of the gate in meters.
  public: double width;

  /// \brief The center of the gate in the world frame. Note that the roll and
  /// pitch are ignored. Only yaw is relevant and it points into the direction
  /// in which the gate should be crossed.
  public: math::Pose3d pose;

  /// \brief The state of this gate.
  public: GateState state = GateState::VEHICLE_OUTSIDE;

  /// \brief The Entity Component Manager.
  public: sim::EntityComponentManager &ecm;
};

/////////////////////////////////////////////////
VirtualGate::VirtualGate(const sim::Entity _leftMarkerEntity,
  const math::Vector3d &_offset, double _width,
  sim::EntityComponentManager &_ecm)
  : leftMarkerEntity(_leftMarkerEntity),
    offset(_offset),
    width(_width),
    ecm(_ecm)
{
  this->Update();
}

/////////////////////////////////////////////////
void VirtualGate::Update()
{
  // The pose of the markers delimiting the virtual gate.
  auto comp =
    this->ecm.Component<sim::components::WorldPose>(this->leftMarkerEntity);

  // Sanity check: Make sure that we can access the component.
  if (!comp)
    return;

  math::Vector3d leftMarkerPos = comp->Data().Pos();
  auto rightMarkerPos = leftMarkerPos + this->offset;

  // Unit vector from the left marker to the right one.
  auto v1 = leftMarkerPos - rightMarkerPos;
  v1.Normalize();

  // Unit vector perpendicular to v1 in the direction we like to cross gates.
  const auto v2 = -math::Vector3d::UnitZ.Cross(v1);

  // This is the center point of the gate.
  const auto middle = (leftMarkerPos + rightMarkerPos) / 2.0;

  // Yaw of the gate in world coordinates.
  const auto yaw = atan2(v2.Y(), v2.X());

  // The updated pose.
  this->pose.Set(middle, math::Vector3d(0, 0, yaw));
}

/////////////////////////////////////////////////
GateState VirtualGate::IsPoseInGate(const math::Pose3d &_robotWorldPose) const
{
  // Transform to gate frame.
  const math::Vector3d robotLocalPosition =
    this->pose.Rot().Inverse().RotateVector(_robotWorldPose.Pos() -
    this->pose.Pos());

  // Are we within the width?
  if (fabs(robotLocalPosition.Y()) <= this->width / 2.0)
  {
    if (robotLocalPosition.X() >= 0.0)
      return GateState::VEHICLE_AFTER;
    else
      return GateState::VEHICLE_BEFORE;
  }
  else
    return GateState::VEHICLE_OUTSIDE;
}

/// \brief A buoy that is part of the wildlife task.
class Buoy
{
  /// \brief Constructor.
  /// \param[in] _entityName The buoy's scoped name.
  /// \param[in] _buoyEntity The buoy's main entity.
  /// \param[in] _buoyGoal The buoy's goal.
  /// \param[in] _engagementDistance The vehicle engages with the buoy when
  ///            the distance between them is lower or equal than this value.
  /// \param[in out] _ecm The Entity Component Manager.
  public: Buoy(const std::string &_entityName,
               const sim::Entity _buoyEntity,
               const BuoyGoal _buoyGoal,
               double _engagementDistance,
               sim::EntityComponentManager &_ecm);

  /// \brief Update the status of this buoy.
  public: void Update();

  /// \brief Set the vehicle model.
  /// \param[in] _vehicleModel The vehicle model entity.
  public: void SetVehicleModel(sim::Entity _vehicleModel);

  /// \brief The number of virtual gates;
  private: const unsigned int kNumVirtualGates = 8u;

  /// \brief The buoy's main entity.
  public: const sim::Entity entity;

  /// \brief The buoy's entity name.
  public: std::string entityName;

  /// \brief The goal.
  public: const BuoyGoal goal;

  /// \brief The vehicle engages with the buoy when the distance between them
  /// is lower or equal than this value.
  public: const double engagementDistance;

  /// \brief The state of this buoy.
  public: BuoyState state = BuoyState::NEVER_ENGAGED;

  /// \brief Entity of the vehicle that interacts with the buoy.
  public: sim::Entity vehicleModel = sim::kNullEntity;

  /// \brief A collection of virtual gates around the buoy.
  public: std::vector<VirtualGate> virtualGates;

  /// \brief The number of virtual gates currently crossed.
  public: unsigned int numVirtualGatesCrossed = 0u;

  /// \brief The Entity Component Manager.
  public: sim::EntityComponentManager &ecm;
};

/////////////////////////////////////////////////
Buoy::Buoy(const std::string &_entityName,
           const sim::Entity _buoyEntity,
           const BuoyGoal _buoyGoal,
           double _engagementDistance,
           sim::EntityComponentManager &_ecm)
  : entityName(_entityName),
    entity(_buoyEntity),
    goal(_buoyGoal),
    engagementDistance(_engagementDistance),
    ecm(_ecm)
{
  if (this->goal == BuoyGoal::AVOID)
    return;

  // Initialize the virtual gates.
  const double kAangleIncrement = 2 * GZ_PI / this->kNumVirtualGates;
  for (int i = 0; i < this->kNumVirtualGates; ++i)
  {
    double alpha = kAangleIncrement * i;
    math::Vector3d offset;
    offset.X(this->engagementDistance * cos(alpha));
    offset.Y(this->engagementDistance * sin(alpha));

    auto comp = this->ecm.Component<sim::components::WorldPose>(this->entity);
    if (!comp)
    {
      gzerr << "Unable to find component WorldPose for entity ["
            << this->entity << "]" << std::endl;
      return;
    }

    math::Pose3d pose = comp->Data();
    offset.Z(pose.Pos().Z());

    this->virtualGates.push_back(
      VirtualGate(_buoyEntity, offset, _engagementDistance, this->ecm));
  }

  this->Update();
}

/////////////////////////////////////////////////
void Buoy::Update()
{
  if (this->state == BuoyState::CIRCUMNAVIGATED ||
      this->vehicleModel == sim::kNullEntity)
  {
    return;
  }

  // Sanity check: Make sure that we can access the component.
  auto comp =
    this->ecm.Component<sim::components::Pose>(this->vehicleModel);
  if (!comp)
  {
    gzerr << "Unable to find Pose for vehicle model [" << this->vehicleModel
          << "]" << std::endl;
    return;
  }

  const math::Pose3d vehiclePose = comp->Data();

  // Sanity check: Make sure that we can access the component.
  auto comp2 = this->ecm.Component<sim::components::WorldPose>(this->entity);
  if (!comp2)
  {
    gzerr << "Unable to find WorldPose for buoy [" << this->entity
          << "]" << std::endl;
    return;
  }

  const math::Pose3d buoyPose = comp2->Data();

  const double vehicleBuoyDistance = vehiclePose.Pos().Distance(buoyPose.Pos());

  if (this->state == BuoyState::NEVER_ENGAGED)
  {
    if (vehicleBuoyDistance <= this->engagementDistance)
    {
      // Transition to ENGAGED.
      this->state = BuoyState::ENGAGED;
      gzdbg << this->entityName << " Transition from NEVER_ENGAGED"
            << " to ENGAGED" << std::endl;
      std::cout << std::flush;
      return;
    }
  }
  else if (this->state == BuoyState::NOT_ENGAGED)
  {
    if (vehicleBuoyDistance <= this->engagementDistance)
    {
      // Transition to ENGAGED.
      this->state = BuoyState::ENGAGED;
      gzdbg << this->entityName << " Transition from NOT_ENGAGED"
            << " to ENGAGED" << std::endl;
      std::cout << std::flush;
      return;
    }
  }
  else if (this->state == BuoyState::ENGAGED)
  {
    if (vehicleBuoyDistance > this->engagementDistance)
    {
      // Transition to NOT ENGAGED.
      this->state = BuoyState::NOT_ENGAGED;
      gzdbg << this->entityName << " Transition from ENGAGED"
            << " to NOT_ENGAGED" << std::endl;
      std::cout << std::flush;

      // You need to start over.
      this->numVirtualGatesCrossed = 0u;
      return;
    }

    if (this->goal == BuoyGoal::AVOID)
      return;

    // Check circumnavigation using the virtual gates.
    for (auto &virtualGate : this->virtualGates)
    {
      virtualGate.Update();

      // Check if we have crossed this gate.
      auto currentState = virtualGate.IsPoseInGate(vehiclePose);
      if (currentState == GateState::VEHICLE_AFTER &&
          virtualGate.state == GateState::VEHICLE_BEFORE)
      {
        currentState = GateState::CROSSED;

        if (this->goal == BuoyGoal::CIRCUMNAVIGATE_COUNTERCLOCKWISE)
        {
          ++this->numVirtualGatesCrossed;
          gzdbg << this->entityName
                << " Virtual gate crossed counterclockwise! ("
                << 100 * this->numVirtualGatesCrossed / this->kNumVirtualGates
                << "% completed)" << std::endl;
          std::cout << std::flush;
        }
        else
        {
          this->numVirtualGatesCrossed = 0u;
          gzdbg << this->entityName
                << " Virtual gate incorrectly crossed counterclockwise! ("
                << 100 * this->numVirtualGatesCrossed / this->kNumVirtualGates
                << "% completed)" << std::endl;
          std::cout << std::flush;
        }
      }
      else if (currentState == GateState::VEHICLE_BEFORE &&
               virtualGate.state == GateState::VEHICLE_AFTER)
      {
        currentState = GateState::CROSSED;

        if (this->goal == BuoyGoal::CIRCUMNAVIGATE_CLOCKWISE)
        {
          ++this->numVirtualGatesCrossed;
          gzdbg << this->entityName
                << " Virtual gate crossed clockwise! ("
                << 100 * this->numVirtualGatesCrossed / this->kNumVirtualGates
                << "% completed)" << std::endl;
          std::cout << std::flush;
        }
        else
        {
          this->numVirtualGatesCrossed = 0u;
          gzdbg << this->entityName
                << " Virtual gate incorrectly crossed clockwise! ("
                << 100 * this->numVirtualGatesCrossed / this->kNumVirtualGates
                << "% completed)" << std::endl;
          std::cout << std::flush;
        }
      }

      if (this->numVirtualGatesCrossed == this->kNumVirtualGates)
        this->state = BuoyState::CIRCUMNAVIGATED;

      virtualGate.state = currentState;
    }
  }
}

/////////////////////////////////////////////////
void Buoy::SetVehicleModel(sim::Entity _vehicleModel)
{
  this->vehicleModel = _vehicleModel;
}

/// \brief Private WildlifeScoringPlugin data class.
class WildlifeScoringPlugin::Implementation
{
  /// \brief Parse the buoys from SDF.
  /// \param[in] _sdf The current SDF element.
  /// \param[in out] _ecm The Entity Component Manager.
  /// \return True when the buoys were successfully parsed or false otherwise.
  public: bool ParseBuoys(sdf::ElementPtr _sdf,
                          sim::EntityComponentManager &_ecm);

  /// \brief Register a new buoy.
  /// \param[in] _name The scoped name of the buoy.
  /// \param[in] _goal The goal associated to this buoy.
  /// \param[in out] _ecm The Entity Component Manager.
  /// \return True when the buoy has been registered or false otherwise.
  public: bool AddBuoy(const std::string &_name,
                       const std::string &_goal,
                       sim::EntityComponentManager &_ecm);

  /// \brief Publish a new ROS message with the animal locations.
  /// \param[in] _info Sim update information.
  /// \paran[in out] _ecm The Entity Component Manager.
  public: void PublishAnimalLocations(const sim::UpdateInfo &_info,
                                      sim::EntityComponentManager &_ecm);

  /// \brief Compute the total bonus achieved.
  /// \return The time bonus in seconds.
  public: double TimeBonus() const;

  /// \brief SDF pointer.
  public: sdf::ElementPtr sdf;

  /// \brief All the buoys.
  public: std::vector<Buoy> buoys;

  /// \brief The name of the topic where the animal locations are published.
  public: std::string topicPrefix = "/vrx/wildlife/animal";

  /// \brief Time bonus granted for each goal achieved.
  public: double timeBonus = 30.0;

  /// \brief When the vehicle is between the buoy and this distance, the vehicle
  /// engages with the buoy.
  public: double engagementDistance = 10.0;

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief Vector of publishers for the animal locations.
  public: std::vector<transport::Node::Publisher> animalPubs;

  /// \brief True when a vehicle collision is detected.
  public: std::atomic<bool> collisionDetected{false};

  /// \brief Spherical coordinates conversions.
  public: math::SphericalCoordinates sc;

  /// \brief Whether we are in the first plugin iteration or not.
  public: bool firstIteration = true;

  /// \brief Animal pose publication frequency (Hz).
  public: double pubFrequency = 1.0;

  /// \brief Time of the last publication sent (sim time).
  public: std::chrono::duration<double> lastPublicationTime{0};
};

//////////////////////////////////////////////////
WildlifeScoringPlugin::WildlifeScoringPlugin()
  : ScoringPlugin(),
    dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void WildlifeScoringPlugin::Configure(const sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  sim::EntityComponentManager &_ecm, sim::EventManager &_eventMgr)
{
  ScoringPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  this->dataPtr->sdf = _sdf->Clone();

  // Initialize spherical coordinates instance.
  auto worldEntity = _ecm.EntityByComponents(sim::components::World());
  sim::World world(worldEntity);
  this->dataPtr->sc = world.SphericalCoordinates(_ecm).value();

  // Parse the optional <topic_prefix> element.
  if (_sdf->HasElement("topic_prefix"))
    this->dataPtr->topicPrefix = _sdf->Get<std::string>("topic_prefix");

  // Parse the optional <engagement_distance> element.
  if (_sdf->HasElement("engagement_distance"))
  {
    this->dataPtr->engagementDistance =
      _sdf->Get<double>("engagement_distance");
  }

  // Parse the optional <time_bonus> element.
  if (_sdf->HasElement("time_bonus"))
    this->dataPtr->timeBonus = _sdf->Get<double>("time_bonus");

  // Parse the optional <publication_frequency> element.
  if (_sdf->HasElement("publication_frequency"))
    this->dataPtr->pubFrequency = _sdf->Get<double>("publication_frequency");

  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;
}

//////////////////////////////////////////////////
void WildlifeScoringPlugin::PreUpdate(const sim::UpdateInfo &_info,
  sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (this->dataPtr->firstIteration)
  {
    this->dataPtr->firstIteration = false;

    // Parse the optional <buoys> element.
    // Note: Parse this element at the end because we use some of the previous
    // parameters.
    if (this->dataPtr->sdf->HasElement("buoys"))
    {
      auto const &buoysElem = this->dataPtr->sdf->GetElement("buoys");
      if (!this->dataPtr->ParseBuoys(buoysElem, _ecm))
      {
        gzerr << "Score has been disabled" << std::endl;
        return;
      }

      // Advertise the topic for each animal.
      for (auto i = 0; i < this->dataPtr->buoys.size(); ++i)
      {
        auto topic = this->dataPtr->topicPrefix + std::to_string(i) + "/pose";
        this->dataPtr->animalPubs.push_back(
          this->dataPtr->node.Advertise<msgs::Pose>(topic));
      };
    }
  }

  ScoringPlugin::PreUpdate(_info, _ecm);

  // The vehicle is not in the simulation yet.
  if (this->VehicleEntity() == sim::kNullEntity)
    return;

  // Skip if we're not in running mode.
  if (this->TaskState() != "running")
    return;

  // Current score.
  this->ScoringPlugin::SetScore(
    std::min(this->RunningStateDuration(), this->ElapsedTime().count()));

  // Update the state of all buoys.
  bool taskCompleted = true;
  for (auto &buoy : this->dataPtr->buoys)
  {
    // Update the vehicle model if needed.
    if (buoy.vehicleModel == sim::kNullEntity)
    {
      sim::enableComponent<sim::components::WorldPose>(
        _ecm, this->VehicleEntity(), true);
      buoy.SetVehicleModel(this->VehicleEntity());
    }

    // If a collision is detected, invalidate the circumnavigations.
    if (this->dataPtr->collisionDetected)
      buoy.numVirtualGatesCrossed = 0u;

    // Update the buoy state.
    buoy.Update();

    // We consider the task completed when all the circumnavigation goals have
    // been completed.
    if ((buoy.goal == BuoyGoal::CIRCUMNAVIGATE_CLOCKWISE ||
         buoy.goal == BuoyGoal::CIRCUMNAVIGATE_COUNTERCLOCKWISE) &&
        buoy.state != BuoyState::CIRCUMNAVIGATED)
    {
      taskCompleted = false;
    }
  }

  this->dataPtr->collisionDetected = false;

  // Publish the location of the buoys.
  this->dataPtr->PublishAnimalLocations(_info, _ecm);

  if (taskCompleted)
  {
    gzmsg << "Course completed!" << std::endl;
    this->SetScore(this->Score() - this->dataPtr->TimeBonus());
    this->Finish();
  }
}

//////////////////////////////////////////////////
bool WildlifeScoringPlugin::Implementation::ParseBuoys(sdf::ElementPtr _sdf,
  sim::EntityComponentManager &_ecm)
{
  // We need at least one buoy.
  if (!_sdf->HasElement("buoy"))
  {
    gzerr << "Unable to find <buoy> element in SDF." << std::endl;
    return false;
  }

  auto buoyElem = _sdf->GetElement("buoy");

  // Parse a new buoy.
  while (buoyElem)
  {
    // The buoy's model.
    if (!buoyElem->HasElement("name"))
    {
      gzerr << "Unable to find <buoys><buoy><name> element in SDF."
            << std::endl;
      return false;
    }

    const std::string buoyModelName = buoyElem->Get<std::string>("name");

    // The buoy's goal.
    if (!buoyElem->HasElement("goal"))
    {
      gzerr << "Unable to find <buoys><buoy><goal> element in SDF."
            << std::endl;
      return false;
    }

    const std::string buoyGoal = buoyElem->Get<std::string>("goal");
    if (!this->AddBuoy(buoyModelName, buoyGoal, _ecm))
      return false;

    // Parse the next buoy.
    buoyElem = buoyElem->GetNextElement("buoy");
  }

  return true;
}

//////////////////////////////////////////////////
bool WildlifeScoringPlugin::Implementation::AddBuoy(const std::string &_name,
  const std::string &_goal, sim::EntityComponentManager &_ecm)
{
  auto entity = sim::entitiesFromScopedName(_name, _ecm);
  if (entity.empty())
  {
    gzerr << "Unable to find entity [" << _name << "]" << std::endl;
    return false;
  }

  BuoyGoal buoyGoal;
  if (_goal == "avoid")
    buoyGoal = BuoyGoal::AVOID;
  else if (_goal == "circumnavigate_clockwise")
    buoyGoal = BuoyGoal::CIRCUMNAVIGATE_CLOCKWISE;
  else if (_goal == "circumnavigate_counterclockwise")
    buoyGoal = BuoyGoal::CIRCUMNAVIGATE_COUNTERCLOCKWISE;
  else
  {
    gzerr << "Unknown <goal> value: [" << _goal << "]" << std::endl;
    return false;
  }

  // Save the new buoy.
  this->buoys.push_back(
    Buoy(_name, *entity.begin(), buoyGoal, this->engagementDistance, _ecm));

  return true;
}

//////////////////////////////////////////////////
void WildlifeScoringPlugin::Implementation::PublishAnimalLocations(
  const sim::UpdateInfo &_info, sim::EntityComponentManager &_ecm)
{
  auto now = _info.simTime;
  std::chrono::duration<double> elapsed = now - this->lastPublicationTime;
  if (std::chrono::duration<double>(elapsed).count() <
      1 / this->pubFrequency)
  {
    return;
  }

  this->lastPublicationTime = now;

  auto stamp = sim::convert<msgs::Time>(_info.simTime);

  uint8_t i = 0u;
  for (auto const &buoy : this->buoys)
  {
    // Conversion from Gazebo Cartesian coordinates to spherical.
    auto comp = _ecm.Component<sim::components::WorldPose>(buoy.entity);
    if (!comp)
    {
      gzerr << "Unable to find WorldPose component for entity ["
            << buoy.entity << "]" << std::endl;
      return;
    }

    math::Pose3d pose = comp->Data();

    auto in = math::SphericalCoordinates::CoordinateType::GLOBAL;
    auto out = math::SphericalCoordinates::CoordinateType::SPHERICAL;
    auto latlon = this->sc.PositionTransform(pose.Pos(), in, out);
    latlon.X(GZ_RTOD(latlon.X()));
    latlon.Y(GZ_RTOD(latlon.Y()));

    const math::Quaternion<double> orientation = pose.Rot();

    // Fill the message.
    msgs::Pose geoPoseMsg;

    // header->stamp
    geoPoseMsg.mutable_header()->mutable_stamp()->CopyFrom(stamp);

    // header->frame_id - We set the buoy type based on its goal.
    auto frame = geoPoseMsg.mutable_header()->add_data();
    frame->set_key("frame_id");

    if (buoy.goal == BuoyGoal::AVOID)
      frame->add_value("crocodile");
    else if (buoy.goal == BuoyGoal::CIRCUMNAVIGATE_CLOCKWISE)
      frame->add_value("platypus");
    else if (buoy.goal == BuoyGoal::CIRCUMNAVIGATE_COUNTERCLOCKWISE)
      frame->add_value("turtle");
    else
      frame->add_value("unknown");

    // pose
    geoPoseMsg.mutable_position()->set_x(latlon.X());
    geoPoseMsg.mutable_position()->set_y(latlon.Y());
    geoPoseMsg.mutable_position()->set_z(latlon.Z());
    geoPoseMsg.mutable_orientation()->set_x(orientation.X());
    geoPoseMsg.mutable_orientation()->set_y(orientation.Y());
    geoPoseMsg.mutable_orientation()->set_z(orientation.Z());
    geoPoseMsg.mutable_orientation()->set_w(orientation.W());

    this->animalPubs[i].Publish(geoPoseMsg);
    ++i;
  }
}

//////////////////////////////////////////////////
double WildlifeScoringPlugin::Implementation::TimeBonus() const
{
  // Check time bonuses.
  double totalBonus = 0;
  for (auto const &buoy : this->buoys)
  {
    if (buoy.goal == BuoyGoal::AVOID &&
        buoy.state == BuoyState::NEVER_ENGAGED)
    {
      totalBonus += this->timeBonus;
    }
    else if ((buoy.goal == BuoyGoal::CIRCUMNAVIGATE_CLOCKWISE         ||
              buoy.goal == BuoyGoal::CIRCUMNAVIGATE_COUNTERCLOCKWISE) &&
             buoy.state == BuoyState::CIRCUMNAVIGATED)
    {
      totalBonus += this->timeBonus;
    }
  }

  return totalBonus;
}

//////////////////////////////////////////////////
void WildlifeScoringPlugin::OnCollision()
{
  gzdbg << "Collision detected, invalidating circumnavigations" << std::endl;
  this->dataPtr->collisionDetected = true;
}

//////////////////////////////////////////////////
void WildlifeScoringPlugin::OnFinished()
{
  this->SetTimeoutScore(this->Score() - this->dataPtr->TimeBonus());
  ScoringPlugin::OnFinished();
}

GZ_ADD_PLUGIN(WildlifeScoringPlugin,
              sim::System,
              WildlifeScoringPlugin::ISystemConfigure,
              WildlifeScoringPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::WildlifeScoringPlugin,
                    "vrx::WildlifeScoringPlugin")
