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

#include <chrono>
#include <string>
#include <vector>
#include <gz/common/Profiler.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

#include "WayfindingScoringPlugin.hh"
#include "WaypointMarkers.hh"

using namespace gz;
using namespace vrx;

/// \brief Private WayfindingScoringPlugin data class.
class WayfindingScoringPlugin::Implementation
{
  /// \brief A transport node.
  public: transport::Node node{transport::NodeOptions()};

  /// \brief Topic where the list of waypoints is published.
  public: std::string waypointsTopic = "/vrx/wayfinding/waypoints";

  /// \brief Topic where the current minimum pose error distance for each
  /// waypoint is published.
  public: std::string minErrorsTopic = "/vrx/wayfinding/min_errors";

  /// \brief Topic where the current average minimum error is published.
  public: std::string meanErrorTopic = "/vrx/wayfinding/mean_error";

  /// \brief Publisher for the goal.
  public: transport::Node::Publisher waypointsPub;

  /// \brief Publisher for the combined 2D pose error.
  public: transport::Node::Publisher minErrorsPub;

  /// \brief Publisher for the current rms error.
  public: transport::Node::Publisher meanErrorPub;

  /// \brief Vector containing waypoints as 3D vectors of doubles representing
  /// X Y yaw, where X and Y are local (Gazebo) coordinates.
  public: std::vector<math::Vector3d> localWaypoints;

  /// \brief Vector containing waypoints as 3D vectors of doubles representing
  /// Lattitude Longitude yaw, where lattitude and longitude are given in
  /// spherical (WGS84) coordinates.
  public: std::vector<math::Vector3d> sphericalWaypoints;

  // For publishing the waypoint locations
  public: msgs::Pose_V waypointsMessage;

  /// \brief Vector containing current minimum 2D pose error achieved for each
  /// waypoint so far.
  public: std::vector<double> minErrors;

  /// \brief Current average minimum error for all waypoints.
  public: double meanError;

  /// \brief Pointer to the SDF plugin element.
  public: sdf::ElementPtr sdf;

  /// \brief Spherical coordinate conversions. 
  public: math::SphericalCoordinates sc; 

  /// \brief Entity of the vehicle used.
  public: sim::Entity vehicleEntity{sim::kNullEntity};

  /// \brief Waypoint visualization markers.
  public: WaypointMarkers waypointMarkers{"waypoint_marker"};
};

//////////////////////////////////////////////////
WayfindingScoringPlugin::WayfindingScoringPlugin()
  : ScoringPlugin(),
    dataPtr(utils::MakeUniqueImpl<Implementation>())
{
  gzmsg << "Wayfinding scoring plugin loaded" << std::endl;
}

//////////////////////////////////////////////////
WayfindingScoringPlugin::~WayfindingScoringPlugin()
{
}

//////////////////////////////////////////////////
void WayfindingScoringPlugin::Configure(const sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  sim::EntityComponentManager &_ecm, sim::EventManager &_eventMgr)
{
  ScoringPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  this->dataPtr->sdf = _sdf->Clone();

  auto worldEntity = _ecm.EntityByComponents(sim::components::World());
  sim::World world(worldEntity);

  this->dataPtr->sc = world.SphericalCoordinates(_ecm).value();

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
    math::Vector3d latlonyaw = waypointElem->Get<math::Vector3d>("pose");
  
    // Convert lat/lon to local
    //  snippet from UUV Simulator SphericalCoordinatesROSInterfacePlugin.cc
    math::Vector3d scVec(latlonyaw.X(), latlonyaw.Y(), 0.0);
  
    math::Vector3d cartVec =
      this->dataPtr->sc.LocalFromSphericalPosition(scVec);
  
    cartVec.Z() = latlonyaw.Z();

    // build message
    math::Pose3d pose(latlonyaw.X(), latlonyaw.Y(), 0, 0, 0, latlonyaw.Z());
    msgs::Set(this->dataPtr->waypointsMessage.add_pose(),pose);
  
    // Set up relevant vectors
    this->dataPtr->sphericalWaypoints.push_back(latlonyaw);
    this->dataPtr->localWaypoints.push_back(cartVec);
  
    // Print some debugging messages
    gzmsg << "Waypoint, Spherical: Lat = " << latlonyaw.X()
           << " Lon = " << latlonyaw.Y() << std::endl;
    gzmsg << "Waypoint, Local: X = " << cartVec.X()
           << " Y = " << cartVec.Y() << " Yaw = " << cartVec.Z() << std::endl;
  
    waypointElem = waypointElem->GetNextElement("waypoint");
  }

  // Throttle messages to 1Hz
  transport::AdvertiseMessageOptions opts;
  opts.SetMsgsPerSec(1u);

  // set up topics
  if (this->dataPtr->sdf->HasElement("waypoints_topic"))
  {
    this->dataPtr->waypointsTopic =
      this->dataPtr->sdf->Get<std::string>("waypoints_topic");
  }

  this->dataPtr->waypointsPub = this->dataPtr->node.Advertise<msgs::Pose_V>(
    this->dataPtr->waypointsTopic, opts);

  if (_sdf->HasElement("min_errors_topic"))
    this->dataPtr->minErrorsTopic = _sdf->Get<std::string>("min_errors_topic");

  this->dataPtr->minErrorsPub = this->dataPtr->node.Advertise<msgs::Float_V>(
    this->dataPtr->minErrorsTopic, opts);

  if (_sdf->HasElement("mean_error_topic"))
    this->dataPtr->meanErrorTopic = _sdf->Get<std::string>("mean_error_topic");

  this->dataPtr->meanErrorPub = this->dataPtr->node.Advertise<msgs::Float>(
    this->dataPtr->meanErrorTopic, opts);

  if (_sdf->HasElement("markers"))
  {
    this->dataPtr->waypointMarkers.Load(
      this->dataPtr->sdf->GetElement("markers"));
    int markerId = 0;
    for (const auto waypoint : this->dataPtr->localWaypoints)
    {
      if (!this->dataPtr->waypointMarkers.DrawMarker(markerId, waypoint.X(),
            waypoint.Y(), waypoint.Z()))
      {
        gzerr << "Error creating visual marker" << std::endl;
      }
      markerId++;
    }
  }
}

//////////////////////////////////////////////////
void WayfindingScoringPlugin::PreUpdate(const sim::UpdateInfo &_info,
  sim::EntityComponentManager &_ecm)
{
  // don't update when paused
  if (_info.paused)
    return;

  ScoringPlugin::PreUpdate(_info,_ecm);

  // Start publishing the goal once in "ready" state
  if (this->ScoringPlugin::TaskState() == "ready" || 
      this->ScoringPlugin::TaskState() == "running")
  {
    this->dataPtr->waypointsPub.Publish(this->dataPtr->waypointsMessage);
  }

  // Nothing else to do if the task is not in "running" state.
  if (this->ScoringPlugin::TaskState() != "running")
    return;

  // Get the vehicle if it's ready
  if (!this->dataPtr->vehicleEntity)
  {
    auto entity = _ecm.EntityByComponents(
      sim::components::Name(ScoringPlugin::VehicleName()));
    if (entity != sim::kNullEntity)
      this->dataPtr->vehicleEntity = entity;
    else
      return;
  }

  // calculate scores
  auto vehiclePose = _ecm.Component<sim::components::Pose>(
    this->dataPtr->vehicleEntity)->Data();

  double currentHeading = vehiclePose.Rot().Euler().Z();

  double currentTotalError = 0;

  for (unsigned i = 0; i < this->dataPtr->localWaypoints.size(); ++i)
  {
    const math::Vector3d wp = this->dataPtr->localWaypoints[i];
    double dx        =  wp.X() - vehiclePose.Pos().X();
    double dy        =  wp.Y() - vehiclePose.Pos().Y();
    double dist      = sqrt(pow(dx, 2) + pow(dy, 2));
    double k         = 0.75;
    double dhdg      = abs(wp.Z() - currentHeading);
    double headError = M_PI - abs(dhdg - M_PI);
    double poseError =  dist + (pow(k, dist) * headError);

    // If this is the first time through, minError == poseError
    if (i == this->dataPtr->minErrors.size())
      this->dataPtr->minErrors.push_back(poseError);

    // If poseError is smaller than the minimum, update the minimum
    if (poseError < this->dataPtr->minErrors.at(i))
      this->dataPtr->minErrors.at(i) = poseError;

    // add current minimum to current total error
    currentTotalError += this->dataPtr->minErrors.at(i);
  }

  this->dataPtr->meanError =
    currentTotalError / this->dataPtr->localWaypoints.size();

  // set up messages
  msgs::Float_V minErrorsMsg;
  msgs::Float meanErrorMsg;

  meanErrorMsg.set_data(this->dataPtr->meanError);
  for (unsigned i = 0; i < this->dataPtr->minErrors.size(); ++i)
    minErrorsMsg.add_data(this->dataPtr->minErrors.at(i));

  // publish
  this->dataPtr->minErrorsPub.Publish(minErrorsMsg);
  this->dataPtr->meanErrorPub.Publish(meanErrorMsg);

  ScoringPlugin::SetScore(this->dataPtr->meanError); 
}

GZ_ADD_PLUGIN(WayfindingScoringPlugin,
              sim::System,
              ScoringPlugin::ISystemConfigure,
              ScoringPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::WayfindingScoringPlugin,
                    "vrx::WayfindingScoringPlugin")
