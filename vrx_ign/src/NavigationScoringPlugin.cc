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
  private: enum class GateState
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

//  /// \brief A gate that is part of the navigation challenge.
//  private: class Gate
//  {
//    /// \brief Constructor.
//    /// \param[in] _leftMarkerName The left marker's model.
//    /// \param[in] _rightMarkerName The right marker's model.
//    public: Gate(const gazebo::physics::LinkPtr _leftMarkerModel,
//                 const gazebo::physics::LinkPtr _rightMarkerModel);
//
//    /// \brief Where is the given robot pose with respect to the gate?
//    /// \param _robotWorldPose Pose of the robot, in the world frame.
//    /// \return The gate state given the current robot pose.
//    public: GateState IsPoseInGate(
//      const math::Pose3d &_robotWorldPose) const;
//
//    /// \brief Recalculate the pose and width of the gate.
//    // TODO: Implement
//    // public: void Update();
//
//    //TODO: check type
//    /// \brief The left marker model.
//    // public: gazebo::physics::LinkPtr leftMarkerModel;
//
//    //TODO: check type
//    /// \brief The right marker model.
//    // public: gazebo::physics::LinkPtr rightMarkerModel;
//
//    /// \brief The center of the gate in the world frame. Note that the roll and
//    /// pitch are ignored. Only yaw is relevant and it points into the direction
//    /// in which the gate should be crossed.
//    public: math::Pose3d pose;
//
//    /// \brief The width of the gate in meters.
//    public: double width;
//
//    /// \brief The state of this gate.
//    public: GateState state = GateState::VEHICLE_OUTSIDE;
//  };

  /// \brief Parse the gates from SDF.
  /// \param[in] _sdf The current SDF element.
  /// \return True when the gates were successfully parsed or false othwerwise.
  // TODO: Implement
  // private: bool ParseGates(sdf::ElementPtr _sdf);

  /// \brief Register a new gate.
  /// \param[in] _leftMarkerName The name of the left marker.
  /// \param[in] _rightMarkerName The name of the right marker.
  /// \return True when the gate has been registered or false otherwise.
  // TODO: Implement
  // private: bool AddGate(const std::string &_leftMarkerName,
  //                      const std::string &_rightMarkerName);

  // TODO: Implement
  /// \brief Set the score to 0 and change to state to "finish".
  // private: void Fail();

  // Name of Course
  // TODO: Check Type 
  // private: gazebo::physics::ModelPtr course;

  /// \brief All the gates.
  // TODO: Implement Gate class 
//  public: std::vector<Gate> gates;

  /// \brief Number of gates
  public: int numGates;

  /// \brief Number of points deducted per collision.
  public: double obstaclePenalty = 10.0;

   /// \brief Entity of the vehicle used.
  public: gazebo::Entity vehicleEntity;

  //TODO: needed?
  /// \brief Display or suppress state changes
  public: bool silent = false;
  
};

//TODO: Get a minimal plugin running
//TODO: below this line is stationkeeping

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
  auto worldEntity =
      _ecm.EntityByComponents(gazebo::components::World());
  gazebo::World world(worldEntity);

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
  if (this->ScoringPlugin::TaskState() != "running")
    return;

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
