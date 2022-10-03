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

#include <vector>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/msgs/float.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/plugin/Register.hh>

#include "NavigationScoringPlugin.hh"

using namespace ignition;
using namespace vrx;

/// \brief Private ScoringPlugin data class.
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
    public: Gate(const std::string _leftMarkerName,
                 const std::string _rightMarkerName);

    /// \brief Where is the given robot pose with respect to the gate?
    /// \param _robotWorldPose Pose of the robot, in the world frame.
    /// \return The gate state given the current robot pose.
    public: GateState IsPoseInGate(
      const math::Pose3d &_robotWorldPose) const;

    /// \brief Recalculate the pose and width of the gate.
    public: void Update(gazebo::EntityComponentManager &_ecm);

    /// \brief Get right and left marker entities
    public: bool LoadEntities(gazebo::EntityComponentManager &_ecm);

    /// \brief The left marker model name.
    public: std::string leftMarkerName;

    /// \brief The right marker model name.
    public: std::string rightMarkerName;

    /// \brief The left marker entity.
    public: gazebo::Entity leftMarkerEntity;

    /// \brief The right marker entity.
    public: gazebo::Entity rightMarkerEntity;

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

   /// \brief Name of the course used.
  public: std::string courseName;

   /// \brief Entity of the course used.
  public: gazebo::Entity courseEntity;

  /// \brief Pointer to the SDF plugin element.
  public: sdf::ElementPtr sdf;

   /// \brief Entity of the vehicle used.
  public: gazebo::Entity vehicleEntity;

  /// \brief All the gates.
  public: std::vector<Gate> gates;

  /// \brief Number of gates
  public: int numGates;

  /// \brief True if all gate entities have been loaded
  public: bool gatesLoaded = false;

  /// \brief Number of points deducted per collision.
  public: double obstaclePenalty = 10.0;

  //TODO: should this be set in ScoringPlugin?
  /// \brief Display or suppress state changes
  public: bool silent = false;
};

/////////////////////////////////////////////////
NavigationScoringPlugin::Implementation::Gate::Gate(
    const std::string _leftMarkerName,
    const std::string _rightMarkerName)
  : leftMarkerName(_leftMarkerName),
    rightMarkerName(_rightMarkerName)
{
}

bool NavigationScoringPlugin::Implementation::Gate::LoadEntities(
    gazebo::EntityComponentManager &_ecm)
{
  leftMarkerEntity = _ecm.EntityByComponents(
      gazebo::components::Name(leftMarkerName));

  // Sanity check: Make sure that the model exists.
  if (leftMarkerEntity == gazebo::kNullEntity)
  {
    ignerr << "Unable to find entity [" << leftMarkerName << "]" << std::endl;
    return false;
  }

  rightMarkerEntity = _ecm.EntityByComponents(
      gazebo::components::Name(rightMarkerName));

  if (rightMarkerEntity == gazebo::kNullEntity)
  {
    ignerr << "Unable to find entity [" << rightMarkerName << "]" << std::endl;
    return false;
  }

  // update pose and width
  this->Update(_ecm);

  return true;
}
//////////////////////////////////////////////////
void NavigationScoringPlugin::Implementation::Gate::Update(
    gazebo::EntityComponentManager &_ecm)
{
  if (!leftMarkerEntity || !rightMarkerEntity)
    return;
  
  // get the course pose
  auto courseEntity = _ecm.Component<gazebo::components::ParentEntity>(
     leftMarkerEntity)->Data();
  auto coursePose = _ecm.Component<gazebo::components::Pose>(
     courseEntity)->Data();

  // The relative pose of the markers delimiting the gate.
  auto leftMarkerPose = _ecm.Component<gazebo::components::Pose>(
    leftMarkerEntity)->Data() + coursePose;
  auto rightMarkerPose = _ecm.Component<gazebo::components::Pose>(
    rightMarkerEntity)->Data()+ coursePose;

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
  this->pose.Set(middle, ignition::math::Vector3d(0, 0, yaw));

  // The updated width.
  this->width = leftMarkerPose.Pos().Distance(rightMarkerPose.Pos());

}
/////////////////////////////////////////////////
//TODO: debug below
//TODO: remove super noisy messages 
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
// TODO: Ignition version of this?
//  GZ_ASSERT(_sdf, "NavigationScoringPlugin::ParseGates(): NULL _sdf pointer");

  // We need at least one gate.
  if (!_sdf->HasElement("gate"))
  {
    ignerr << "Unable to find <gate> element in SDF." << std::endl;
    return false;
  }

  auto gateElem = _sdf->GetElement("gate");

  // Parse a new gate.
  while (gateElem)
  {
    // The left marker's name.
    if (!gateElem->HasElement("left_marker"))
    {
      ignerr << "Unable to find <left_marker> element in SDF." << std::endl;
      return false;
    }

    const std::string leftMarkerName =
      gateElem->Get<std::string>("left_marker");

    // The right marker's name.
    if (!gateElem->HasElement("right_marker"))
    {
      ignerr << "Unable to find <right_marker> element in SDF." << std::endl;
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
  ignmsg << "Navigation scoring plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void NavigationScoringPlugin::Configure(const gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gazebo::EntityComponentManager &_ecm, gazebo::EventManager &_eventMgr)
{
  ScoringPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  ignmsg << "Task [" << this->TaskName() << "]" << std::endl;

  this->dataPtr->sdf = _sdf->Clone();

  auto worldEntity =
      _ecm.EntityByComponents(gazebo::components::World());
  gazebo::World world(worldEntity);

  // course_name is a required element.
  if (!_sdf->HasElement("course_name"))
  {
    ignerr << "Unable to find <course_name> element in SDF." << std::endl;
    return;
  }
    this->dataPtr->courseName = _sdf->Get<std::string>("course_name");

  // Optional.
  if (_sdf->HasElement("obstacle_penalty"))
    this->dataPtr->obstaclePenalty = _sdf->Get<double>("obstacle_penalty");

  // This is a required element.
  if (!_sdf->HasElement("gates"))
  {
    ignerr << "Unable to find <gates> element in SDF." << std::endl;
    return;
  }

  // Parse all the gates.
  auto const &gatesElem = this->dataPtr->sdf->GetElement("gates");
  if (!this->dataPtr->ParseGates(gatesElem))
  {
    ignerr << "Score has been disabled" << std::endl;
    return;
  }

  // Save number of gates
  this->dataPtr->numGates = this->dataPtr->gates.size();

  // Set default score in case of timeout.
  double timeoutScore = 200;
  ignmsg << "Setting timeoutScore = " << timeoutScore << std::endl;
  this->ScoringPlugin::SetTimeoutScore(timeoutScore);

  ignmsg << "Task [" << this->TaskName() << "]" << std::endl;

}

//////////////////////////////////////////////////
void NavigationScoringPlugin::PreUpdate( const gazebo::UpdateInfo &_info,
  gazebo::EntityComponentManager &_ecm)
{
  ScoringPlugin::PreUpdate(_info, _ecm);

  // The vehicle might not be ready yet, let's try to get it.
  if (!this->dataPtr->vehicleEntity)
  {
    auto entity = _ecm.EntityByComponents(
      gazebo::components::Name(ScoringPlugin::VehicleName()));
    if (entity != gazebo::kNullEntity)
      this->dataPtr->vehicleEntity = entity;
    else
      return;
  }
  // The course might not be ready yet, let's try to get it.
  if (!this->dataPtr->courseEntity)
  {
    auto entity = _ecm.EntityByComponents(
      gazebo::components::Name(this->dataPtr->courseName));
    if (entity != gazebo::kNullEntity)
      this->dataPtr->courseEntity = entity;
    else
      return;
  }

  // Try to get the gate entities 
  if (!dataPtr->gatesLoaded)
  {
    ignmsg << "Loading " << dataPtr->numGates << " gates." << std::endl; 
    dataPtr->gatesLoaded = true;
    auto iter = std::begin(dataPtr->gates);
    while (iter != std::end(dataPtr->gates))
    {
      Implementation::Gate &gate = *iter;

      dataPtr->gatesLoaded = dataPtr->gatesLoaded && gate.LoadEntities(_ecm);
      ++iter;
    }
    if (!dataPtr->gatesLoaded)
      return;
  }

  if (this->ScoringPlugin::TaskState() != "running")
    return;

  // Current score
  this->ScoringPlugin::SetScore(std::min(this->RunningStateDuration(),
    std::chrono::duration<double>(this->ElapsedTime()).count() +
    this->NumCollisions() * this->dataPtr->obstaclePenalty) / this->dataPtr->numGates);

  auto vehiclePose = _ecm.Component<gazebo::components::Pose>(
    this->dataPtr->vehicleEntity)->Data();

  // Update the state of all gates.
  auto iter = std::begin(this->dataPtr->gates);
  while (iter != std::end(this->dataPtr->gates))
  {
    Implementation::Gate &gate = *iter;

    // Update this gate (in case it moved).
    gate.Update(_ecm);

    // Check if we have crossed this gate.
    auto currentState = gate.IsPoseInGate(vehiclePose);
    if (currentState == Implementation::GateState::VEHICLE_AFTER &&
        gate.state   == Implementation::GateState::VEHICLE_BEFORE)
    {
      currentState = Implementation::GateState::CROSSED;
      ignmsg << "New gate crossed!" << std::endl;

      // We need to cross all gates in order.
      if (iter != this->dataPtr->gates.begin())
      {
        ignmsg << "Gate crossed in the wrong order" << std::endl;
        this->Fail();
        return;
      }

      iter = this->dataPtr->gates.erase(iter);
    }
    // Just checking: did we go backward through the gate?
    else if (currentState == Implementation::GateState::VEHICLE_BEFORE &&
              gate.state   == Implementation::GateState::VEHICLE_AFTER)
     {
       gate.state = Implementation::GateState::INVALID;
       ignmsg << "Transited the gate in the wrong direction. Gate invalidated!"
             << std::endl;
       this->Fail();
       return;
    }
    else
      ++iter;

    gate.state = currentState;
  }

  // Course completed!
  if (this->dataPtr->gates.empty())
  {
    ignmsg << "Course completed!" << std::endl;
    ScoringPlugin::Finish();
  } 
}

//////////////////////////////////////////////////
void NavigationScoringPlugin::Fail()
{
  ScoringPlugin::SetScore(ScoringPlugin::TimeoutScore());
  ScoringPlugin::Finish();
}

//////////////////////////////////////////////////
void NavigationScoringPlugin::OnReady()
{
  if (!this->dataPtr->silent)
  {
    ignmsg << "NavigationScoringPlugin::OnReady" << std::endl;
  }
  ScoringPlugin::OnReady();
}

//////////////////////////////////////////////////
void NavigationScoringPlugin::OnRunning()
{
  if (!this->dataPtr->silent)
  {
    ignmsg << "NavigationScoringPlugin::OnRunning" << std::endl;
  }
  ScoringPlugin::OnRunning();
}

//////////////////////////////////////////////////
void NavigationScoringPlugin::OnFinished()
{
  if (!this->dataPtr->silent)
  {
    ignmsg << "NavigationScoringPlugin::OnFinished" << std::endl;
  }
  ScoringPlugin::OnFinished();
}

//////////////////////////////////////////////////
void NavigationScoringPlugin::OnCollision()
{
  if (!this->dataPtr->silent)
  {
    ignmsg << "NavigationScoringPlugin::OnCollision" << std::endl;
  }
  ScoringPlugin::OnCollision();
}

IGNITION_ADD_PLUGIN(vrx::NavigationScoringPlugin,
                    gazebo::System,
                    vrx::NavigationScoringPlugin::ISystemConfigure,
                    vrx::NavigationScoringPlugin::ISystemPreUpdate)
