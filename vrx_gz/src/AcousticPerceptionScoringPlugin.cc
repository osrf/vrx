/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/World.hh>

#include "AcousticPerceptionScoringPlugin.hh"
#include "WaypointMarkers.hh"

using namespace gz;
using namespace vrx;

/// \brief Private ScoringPlugin data class.
class AcousticPerceptionScoringPlugin::Implementation
{
  /// \brief If the distance between the WAM-V and the pinger is within this
  /// tolerance we consider the task completed.
  public: double goalTolerance = 1;

  /// \brief Waypoint visualization markers
  public: WaypointMarkers waypointMarkers{"pinger_marker"};

  /// \brief Entity of the vehicle used.
  public: sim::Entity vehicleEntity;

  /// \brief The position of the pinger.
  public: math::Vector3d pingerPosition;
};

/////////////////////////////////////////////////
AcousticPerceptionScoringPlugin::AcousticPerceptionScoringPlugin()
  : ScoringPlugin(),
  dataPtr(utils::MakeUniqueImpl<Implementation>())
{
  gzmsg << "AcousticPerceptionScoringPlugin scoring plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void AcousticPerceptionScoringPlugin::Configure(const sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  sim::EntityComponentManager &_ecm, sim::EventManager &_eventMgr)
{
  ScoringPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  // Required pinger position.
  if (!_sdf->HasElement("pinger_position"))
  {
    gzerr << "Unable to find <pinger_position>" << std::endl;
    return;
  }

  this->dataPtr->pingerPosition = _sdf->Get<math::Vector3d>("pinger_position");

  // Optional tolerance.
  if (_sdf->HasElement("goal_tolerance"))
    this->dataPtr->goalTolerance = _sdf->Get<double>("goal_tolerance");

  // Optional pinger marker.
  if (_sdf->HasElement("markers"))
  {
    auto markersElement = _sdf->Clone()->GetElement("markers");

    this->dataPtr->waypointMarkers.Load(markersElement);
    if (!this->dataPtr->waypointMarkers.DrawMarker(0,
           this->dataPtr->pingerPosition.X(), this->dataPtr->pingerPosition.Y(),
           this->dataPtr->pingerPosition.Z()))
    {
      gzerr << "Error creating visual marker" << std::endl;
    }
  }
}

//////////////////////////////////////////////////
void AcousticPerceptionScoringPlugin::PreUpdate( const sim::UpdateInfo &_info,
  sim::EntityComponentManager &_ecm)
{
  // Don't update when paused
  if (_info.paused)
    return;

  ScoringPlugin::PreUpdate(_info, _ecm);

  this->SetScore(this->ElapsedTime().count());

  // The vehicle might not be ready yet, let's try to get it.
  if (!this->dataPtr->vehicleEntity)
  {
    auto entity = _ecm.EntityByComponents(
      sim::components::Name(ScoringPlugin::VehicleName()));
    if (entity == sim::kNullEntity)
      return;

    this->dataPtr->vehicleEntity = entity;
  }

  auto vehiclePose = _ecm.Component<sim::components::Pose>(
    this->dataPtr->vehicleEntity)->Data();

  double distance = vehiclePose.Pos().Distance(this->dataPtr->pingerPosition);

  if (distance <= this->dataPtr->goalTolerance)
    this->Finish();
}

GZ_ADD_PLUGIN(vrx::AcousticPerceptionScoringPlugin,
              sim::System,
              vrx::AcousticPerceptionScoringPlugin::ISystemConfigure,
              vrx::AcousticPerceptionScoringPlugin::ISystemPreUpdate)
