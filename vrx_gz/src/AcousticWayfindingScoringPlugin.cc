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

#include "AcousticWayfindingScoringPlugin.hh"
#include "WaypointMarkers.hh"

using namespace gz;
using namespace vrx;

/// \brief Private ScoringPlugin data class.
class AcousticWayfindingScoringPlugin::Implementation
{
  /// \brief A transport node.
  public: transport::Node node;

  /// \brief Pointer to the SDF plugin element.
  public: sdf::ElementPtr sdf;

  /// \brief If the distance between the WAM-V and the pinger is within this
  /// tolerance we consider the task completed.
  public: double goalTolerance = 1;

  /// \brief Waypoint visualization markers
  public: WaypointMarkers waypointMarkers{"pinger_marker"};

  /// \brief Entity of the vehicle used.
  public: sim::Entity vehicleEntity;

  /// \brief The collection of waypoints.
  public: std::vector<math::Vector3d> waypoints;

  // For publishing the current waypoint location.
  public: msgs::Pose waypointMessage;

  /// \brief Publisher for the pinger location.
  public: transport::Node::Publisher waypointPub;

  /// /brief The topic used to set the pinger position.
  public: std::string setPingerTopicName =
    "/wamv/pingers/pinger/set_pinger_position";
};

/////////////////////////////////////////////////
AcousticWayfindingScoringPlugin::AcousticWayfindingScoringPlugin()
  : ScoringPlugin(),
  dataPtr(utils::MakeUniqueImpl<Implementation>())
{
  gzmsg << "AcousticWayfindingScoringPlugin scoring plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void AcousticWayfindingScoringPlugin::Configure(const sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  sim::EntityComponentManager &_ecm, sim::EventManager &_eventMgr)
{
  ScoringPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  this->dataPtr->sdf = _sdf->Clone();

  // A waypoints element is required.
  if (!this->dataPtr->sdf->HasElement("waypoints"))
  {
    gzerr << "Unable to find <waypoints> element in SDF." << std::endl;
    return;
  }
  auto waypointsElem = this->dataPtr->sdf->GetElement("waypoints");
 
  // We need at least one waypoint
  if (!waypointsElem->HasElement("waypoint"))
  {
    gzerr << "Unable to find <waypoint> element in <waypoints>." << std::endl;
    return;
  }
  auto waypointElem = waypointsElem->GetElement("waypoint");
 
  while (waypointElem)
  {
    math::Vector3d waypoint = waypointElem->Get<math::Vector3d>("pose");
    this->dataPtr->waypoints.push_back(waypoint);
    waypointElem = waypointElem->GetNextElement("waypoint");
  }

  // Optional set pinger position topic name.
  if (_sdf->HasElement("set_pinger_position_topic"))
  {
    this->dataPtr->setPingerTopicName =
      _sdf->Get<std::string>("set_pinger_position_topic");
  }

  // Optional tolerance.
  if (_sdf->HasElement("goal_tolerance"))
    this->dataPtr->goalTolerance = _sdf->Get<double>("goal_tolerance");

  // Optional pinger marker.
  if (_sdf->HasElement("markers"))
  {
    auto markersElement = _sdf->Clone()->GetElement("markers");

    int markerId = 0;
    for (const auto waypoint : this->dataPtr->waypoints)
    {
      if (!this->dataPtr->waypointMarkers.DrawMarker(markerId, waypoint.X(),
            waypoint.Y(), waypoint.Z(), std::to_string(markerId)))
      {
        gzerr << "Error creating visual marker" << std::endl;
      }
      markerId++;
    }
  }

  // Throttle messages to 1Hz.
  transport::AdvertiseMessageOptions opts;
  opts.SetMsgsPerSec(1u);

  // The publisher to update the pinger position.
  this->dataPtr->waypointPub = this->dataPtr->node.Advertise<msgs::Vector3d>(
    this->dataPtr->setPingerTopicName, opts);
}

//////////////////////////////////////////////////
void AcousticWayfindingScoringPlugin::PreUpdate( const sim::UpdateInfo &_info,
  sim::EntityComponentManager &_ecm)
{
  // Don't update when paused
  if (_info.paused)
    return;

  ScoringPlugin::PreUpdate(_info, _ecm);

  // Time to finish the task if there are no waypoints.
  if (this->dataPtr->waypoints.empty())
  {
    this->SetScore(this->ElapsedTime().count());
    this->Finish();
  }

  // The vehicle might not be ready yet, let's try to get it.
  if (!this->dataPtr->vehicleEntity)
  {
    auto entity = _ecm.EntityByComponents(
      sim::components::Name(ScoringPlugin::VehicleName()));
    if (entity == sim::kNullEntity)
      return;

    this->dataPtr->vehicleEntity = entity;
  }

  // Update the pinger position.
  auto pingerPosition = this->dataPtr->waypoints.at(0);
  msgs::Vector3d msg;
  msg.set_x(pingerPosition.X());
  msg.set_y(pingerPosition.Y());
  msg.set_z(pingerPosition.Z());
  this->dataPtr->waypointPub.Publish(msg);

  auto vehiclePose = _ecm.Component<sim::components::Pose>(
    this->dataPtr->vehicleEntity)->Data();

  double distance = vehiclePose.Pos().Distance(pingerPosition);

  if (distance <= this->dataPtr->goalTolerance)
  {
    // Let's enable the next waypoint.
    this->dataPtr->waypoints.erase(this->dataPtr->waypoints.begin());
  }
}

GZ_ADD_PLUGIN(vrx::AcousticWayfindingScoringPlugin,
              sim::System,
              vrx::AcousticWayfindingScoringPlugin::ISystemConfigure,
              vrx::AcousticWayfindingScoringPlugin::ISystemPreUpdate)
