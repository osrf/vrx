/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <gz/msgs/float.pb.h>
#include <gz/msgs/pose.pb.h>
#include <string>
#include <vector>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/World.hh>
#include "NavigationScoringPlugin.hh"

using namespace gz;
using namespace vrx;

/// \brief Private NavigationScoringPlugin data class.
class NavigationScoringPlugin::Implementation
{
  /// \brief All gate states.
  public: enum class GateState
  {
    /// \brief Not "in" the gate.
    VEHICLE_OUTSIDE,

    /// \brief Before the gate.
    VEHICLE_BEFORE,

    /// \brief After the gate.
    VEHICLE_AFTER,

    /// \brief Gate crossed!
    CROSSED,

    /// \brief Gate invalid. E.g.: if crossed in the wrong direction.
    INVALID,
  };

  /// \brief A gate that is part of the navigation challenge.
  public: class Gate
  {
    /// \brief Constructor.
    /// \param[in] _leftMarkerName The left marker's model name.
    /// \param[in] _rightMarkerName The right marker's model name.
    public: Gate(const std::string &_leftMarkerName,
                 const std::string &_rightMarkerName);

    /// \brief Where is the given robot pose with respect to the gate?
    /// \param _robotWorldPose Pose of the robot, in the world frame.
    /// \return The gate state given the current robot pose.
    public: GateState IsPoseInGate(const math::Pose3d &_robotWorldPose) const;

    /// \brief Recalculate the pose and width of the gate.
    /// \param[in] _ecm The ECM.
    public: void Update(sim::EntityComponentManager &_ecm);

    /// \brief Get right and left marker entities
    /// \param[in] _ecm The ECM.
    public: bool LoadEntities(sim::EntityComponentManager &_ecm);

    /// \brief The left marker model name.
    public: std::string leftMarkerName;

    /// \brief The right marker model name.
    public: std::string rightMarkerName;

    /// \brief The left marker entity.
    public: sim::Entity leftMarkerEntity;

    /// \brief The right marker entity.
    public: sim::Entity rightMarkerEntity;

    /// \brief The center of the gate in the world frame. Note that the roll and
    /// pitch are ignored. Only yaw is relevant and it points into the direction
    /// in which the gate should be crossed.
    public: math::Pose3d pose;

    /// \brief The width of the gate in meters.
    public: double width;

    /// \brief The state of this gate.
    public: GateState state = GateState::VEHICLE_OUTSIDE;
  };

  /// \brief Parse the gates from SDF.
  /// \param[in] _sdf The current SDF element.
  /// \return True when the gates were successfully parsed or false othwerwise.
  public: bool ParseGates(sdf::ElementPtr _sdf);

  /// \brief Parse the gate scoring bonus from SDF.
  /// \param[in] _sdf The current SDF element.
  /// \return True when the bonus was successfully parsed or false othwerwise.
  public: bool ParseGateBonus(sdf::ElementPtr _sdf);

   /// \brief Name of the course used.
  public: std::string courseName;

   /// \brief Entity of the course used.
  public: sim::Entity courseEntity;

  /// \brief Pointer to the SDF plugin element.
  public: sdf::ElementPtr sdf;

   /// \brief Entity of the vehicle used.
  public: sim::Entity vehicleEntity;

  /// \brief All the gates.
  public: std::vector<Gate> gates;

  /// \brief Number of gates
  public: int numGates;

  /// \brief True if all gate entities have been loaded
  public: bool gatesLoaded = false;

  /// \brief Number of points deducted per collision.
  public: double obstaclePenalty = 3.0;

  /// \brief Number of points granted for crossing any gate.
  public: double pointsPerGateCrossed = 10.0;

  /// \brief Bonus points granted for crossing two consecutive gates.
  public: double bonusConsecutiveGate = 1.0;

  /// \brief Display or suppress state changes
  public: bool silent = false;

  /// \brief The index of the last gate crossed.
  public: int lastGateCrossed = -1;

  /// \brief The index of the last gate successfully crossed.
  public: int lastGateSuccessfullyCrossed = std::numeric_limits<int>::min();

  /// \brief Previous number of collisions.
  public: int prevCollisions = 0;
};

/////////////////////////////////////////////////
NavigationScoringPlugin::Implementation::Gate::Gate(
    const std::string &_leftMarkerName,
    const std::string &_rightMarkerName)
  : leftMarkerName(_leftMarkerName),
    rightMarkerName(_rightMarkerName)
{
}

/////////////////////////////////////////////////
bool NavigationScoringPlugin::Implementation::Gate::LoadEntities(
    sim::EntityComponentManager &_ecm)
{
  this->leftMarkerEntity = _ecm.EntityByComponents(
    sim::components::Name(this->leftMarkerName));

  // Sanity check: Make sure that the model exists.
  if (this->leftMarkerEntity == sim::kNullEntity)
  {
    gzerr << "Unable to find entity [" << this->leftMarkerName << "]"
          << std::endl;
    return false;
  }

  this->rightMarkerEntity = _ecm.EntityByComponents(
    sim::components::Name(this->rightMarkerName));

  if (this->rightMarkerEntity == sim::kNullEntity)
  {
    gzerr << "Unable to find entity [" << this->rightMarkerName << "]"
          << std::endl;
    return false;
  }

  // update pose and width
  this->Update(_ecm);

  return true;
}
//////////////////////////////////////////////////
void NavigationScoringPlugin::Implementation::Gate::Update(
    sim::EntityComponentManager &_ecm)
{
  if (!this->leftMarkerEntity || !this->rightMarkerEntity)
    return;
  
  // get the course pose
  auto courseEntity = _ecm.Component<sim::components::ParentEntity>(
    this->leftMarkerEntity)->Data();
  auto coursePose = _ecm.Component<sim::components::Pose>(
    courseEntity)->Data();

  // The relative pose of the markers delimiting the gate.
  auto leftMarkerPose =  coursePose * _ecm.Component<sim::components::Pose>(
    this->leftMarkerEntity)->Data();
  auto rightMarkerPose = coursePose * _ecm.Component<sim::components::Pose>(
    this->rightMarkerEntity)->Data();

  // Unit vector from the left marker to the right one.
  auto v1 = leftMarkerPose.Pos() - rightMarkerPose.Pos();
  v1.Normalize();

  // Unit vector perpendicular to v1 in the direction we like to cross gates.
  const auto v2 = math::Vector3d::UnitZ.Cross(v1);

  // This is the center point of the gate.
  const auto middle = (leftMarkerPose.Pos() + rightMarkerPose.Pos()) / 2.0;

  // Yaw of the gate in world coordinates.
  const auto yaw = atan2(v2.Y(), v2.X());

  // The updated pose.
  this->pose.Set(middle, gz::math::Vector3d(0, 0, yaw));

  // The updated width.
  this->width = leftMarkerPose.Pos().Distance(rightMarkerPose.Pos());

}

/////////////////////////////////////////////////
NavigationScoringPlugin::Implementation::GateState 
    NavigationScoringPlugin::Implementation::Gate::IsPoseInGate(
        const math::Pose3d &_robotWorldPose) const
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

//////////////////////////////////////////////////
bool NavigationScoringPlugin::Implementation::ParseGates(sdf::ElementPtr _sdf)
{
  // We need at least one gate.
  if (!_sdf->HasElement("gate"))
  {
    gzerr << "Unable to find <gate> element in SDF." << std::endl;
    return false;
  }

  auto gateElem = _sdf->GetElement("gate");

  // Parse a new gate.
  while (gateElem)
  {
    // The left marker's name.
    if (!gateElem->HasElement("left_marker"))
    {
      gzerr << "Unable to find <left_marker> element in SDF." << std::endl;
      return false;
    }

    const std::string leftMarkerName =
      gateElem->Get<std::string>("left_marker");

    // The right marker's name.
    if (!gateElem->HasElement("right_marker"))
    {
      gzerr << "Unable to find <right_marker> element in SDF." << std::endl;
      return false;
    }

    const std::string rightMarkerName =
      gateElem->Get<std::string>("right_marker");

    // Save the new gate.
    this->gates.push_back(Gate(leftMarkerName, rightMarkerName));

    // Parse the next gate.
    gateElem = gateElem->GetNextElement("gate");
  }

  return true;
}

/////////////////////////////////////////////////
NavigationScoringPlugin::NavigationScoringPlugin()
  : ScoringPlugin(),
  dataPtr(utils::MakeUniqueImpl<Implementation>())
{
  gzmsg << "Navigation scoring plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void NavigationScoringPlugin::Configure(const sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  sim::EntityComponentManager &_ecm, sim::EventManager &_eventMgr)
{
  ScoringPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  this->dataPtr->sdf = _sdf->Clone();

  auto worldEntity = _ecm.EntityByComponents(sim::components::World());
  sim::World world(worldEntity);

  // course_name is a required element.
  if (!_sdf->HasElement("course_name"))
  {
    gzerr << "Unable to find <course_name> element in SDF." << std::endl;
    return;
  }
    this->dataPtr->courseName = _sdf->Get<std::string>("course_name");

  // Optional.
  if (_sdf->HasElement("obstacle_penalty"))
    this->dataPtr->obstaclePenalty = _sdf->Get<double>("obstacle_penalty");

  // Optional.
  if (_sdf->HasElement("points_per_gate_crossed"))
  {
    this->dataPtr->pointsPerGateCrossed =
      _sdf->Get<double>("points_per_gate_crossed");
  }

  // Optional.
  if (_sdf->HasElement("bonus"))
    this->dataPtr->bonusConsecutiveGate =  _sdf->Get<double>("bonus");

  // This is a required element.
  if (!_sdf->HasElement("gates"))
  {
    gzerr << "Unable to find <gates> element in SDF." << std::endl;
    return;
  }

  // Parse all the gates.
  auto const &gatesElem = this->dataPtr->sdf->GetElement("gates");
  if (!this->dataPtr->ParseGates(gatesElem))
  {
    gzerr << "Unable to parse gates. Score has been disabled" << std::endl;
    return;
  }

  // Save number of gates
  this->dataPtr->numGates = this->dataPtr->gates.size();

  // This is a required element.
  if (!_sdf->HasElement("bonus"))
  {
    gzerr << "Unable to find <bonus> element in SDF." << std::endl;
    return;
  }

  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;
}

//////////////////////////////////////////////////
void NavigationScoringPlugin::PreUpdate( const sim::UpdateInfo &_info,
  sim::EntityComponentManager &_ecm)
{
  // don't update when paused
  if (_info.paused)
    return;

  ScoringPlugin::PreUpdate(_info, _ecm);
  // The vehicle might not be ready yet, let's try to get it.
  if (!this->dataPtr->vehicleEntity)
  {
    auto entity = _ecm.EntityByComponents(
      sim::components::Name(ScoringPlugin::VehicleName()));
    if (entity == sim::kNullEntity)
      return;

    this->dataPtr->vehicleEntity = entity;
  }

  // The course might not be ready yet, let's try to get it.
  if (!this->dataPtr->courseEntity)
  {
    auto entity = _ecm.EntityByComponents(
      sim::components::Name(this->dataPtr->courseName));
    if (entity == sim::kNullEntity)
      return;

    this->dataPtr->courseEntity = entity;
  }

  // Try to get the gate entities 
  if (!this->dataPtr->gatesLoaded)
  {
    gzmsg << "Loading " << this->dataPtr->numGates << " gates." << std::endl;
    this->dataPtr->gatesLoaded = true;
    auto iter = std::begin(this->dataPtr->gates);
    while (iter != std::end(this->dataPtr->gates))
    {
      Implementation::Gate &gate = *iter;

      this->dataPtr->gatesLoaded =
        this->dataPtr->gatesLoaded && gate.LoadEntities(_ecm);
      ++iter;
    }
    if (!this->dataPtr->gatesLoaded)
      return;
  }

  if (this->ScoringPlugin::TaskState() != "running")
    return;

  // Compute collisions.
  auto newCollisions = this->NumCollisions() - this->dataPtr->prevCollisions;
  if (newCollisions > 0)
  {
    auto newScore =
      this->Score() - newCollisions * this->dataPtr->obstaclePenalty;
    newScore = std::max(0.0, newScore);
    this->SetScore(newScore);
    this->dataPtr->prevCollisions = this->NumCollisions();
    gzdbg << "New penalty. score: " << this->Score() << std::endl;
    std::cout << std::flush;
  }

  auto vehiclePose = _ecm.Component<sim::components::Pose>(
    this->dataPtr->vehicleEntity)->Data();

  // Update the state of all gates.
  for (auto i = this->dataPtr->lastGateCrossed + 1;
       i < this->dataPtr->numGates; ++i)
  {
    Implementation::Gate &gate = this->dataPtr->gates[i];

    // Update this gate (in case it moved).
    gate.Update(_ecm);

    // Check if we have crossed this gate.
    auto currentState = gate.IsPoseInGate(vehiclePose);

    // Ignore if we haven't crossed the first gate yet.
    if (i != 0 &&
        this->dataPtr->gates[0].state != Implementation::GateState::CROSSED)
    {
      break;
    }

    // Ignore a gate marked as invalid or already crossed.
    if (gate.state == Implementation::GateState::INVALID || 
        gate.state == Implementation::GateState::CROSSED)
    {
      continue;
    }

    // Gate crossed.
    else if (currentState == Implementation::GateState::VEHICLE_AFTER &&
             gate.state == Implementation::GateState::VEHICLE_BEFORE)
    {
      gate.state = Implementation::GateState::CROSSED;
      gzdbg << "New gate crossed!" << std::endl;
      std::cout << std::flush;

      // Add the fix number of points for crossing a gate.
      this->SetScore(this->Score() + this->dataPtr->pointsPerGateCrossed);

      // Add a bonus for crossing two consecutive gates.
      if (i != 0 && i == this->dataPtr->lastGateSuccessfullyCrossed + 1)
      {
        gzdbg << "  Consecutive gate bonus!" << std::endl;
        this->SetScore(this->Score() + this->dataPtr->bonusConsecutiveGate);
      }

      this->dataPtr->lastGateCrossed = i;
      this->dataPtr->lastGateSuccessfullyCrossed = i;

      gzdbg << "Score: " << this->Score() << std::endl;
      std::cout << std::flush;

      // Course completed!
      if (i == this->dataPtr->numGates - 1)
      {
        this->SetScore(this->Score() + this->dataPtr->bonusConsecutiveGate);
        gzdbg << "Extra bonus for completing the course" << std::endl;
        gzdbg << "Score: " << this->Score() << std::endl;
        gzdbg << "Course completed!" << std::endl;
        std::cout << std::flush;
        ScoringPlugin::Finish();
      }

      return;
    }
    // Just checking: did we go backwards through the gate?
    else if (i != 0 &&
             (currentState == Implementation::GateState::VEHICLE_BEFORE &&
              gate.state   == Implementation::GateState::VEHICLE_AFTER))
    {
      gate.state = Implementation::GateState::INVALID;
      gzdbg << "Transited the gate in the wrong direction. Gate invalidated!"
            << std::endl;
      std::cout << std::flush;
      this->dataPtr->lastGateSuccessfullyCrossed =
        std::numeric_limits<int>::min();
      this->dataPtr->lastGateCrossed = i;

      // Course completed!
      if (i == this->dataPtr->numGates - 1)
      {
        gzdbg << "Course completed!" << std::endl;
        std::cout << std::flush;
        ScoringPlugin::Finish();
      }

      return;
    }

    gate.state = currentState;
  }
}

GZ_ADD_PLUGIN(vrx::NavigationScoringPlugin,
                    sim::System,
                    vrx::NavigationScoringPlugin::ISystemConfigure,
                    vrx::NavigationScoringPlugin::ISystemPreUpdate)
