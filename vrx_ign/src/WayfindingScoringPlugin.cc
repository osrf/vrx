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

// #include <geographic_msgs/GeoPoseStamped.h>
// #include <geographic_msgs/GeoPath.h>
// #include <std_msgs/Float64.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <std_msgs/String.h>
// #include <cmath>
// #include <gazebo/common/Console.hh>
// #include <ignition/math/Quaternion.hh>
// #include <ignition/math/Vector3.hh>
// #include <gazebo/physics/Model.hh>
#include <ignition/msgs/param.pb.h>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/World.hh>
#include <chrono>
#include <string>
#include <vector>
#include <ignition/common/Profiler.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/transport/Node.hh>
#include <sdf/sdf.hh>

#include <ignition/plugin/Register.hh>

#include "WayfindingScoringPlugin.hh"
// TODO: uncomment after fixing WaypointMarkers.hh 
// #include "WaypointMarkers.hh"
using namespace ignition;
using namespace vrx;

class WayfindingScoringPlugin::Implementation
{
  public: transport::Node node;

  /// \brief Topic where the list of waypoints is published.
  public: std::string waypointsTopic = "/vrx/wayfinding/waypoints";

  /// \brief Topic where the current minimum pose error distance for each
  /// waypoint is published.
  public: std::string minErrorsTopic = "/vrx/wayfinding/min_errors";

  /// \brief Topic where the current average minimum error is published.
  public: std::string meanErrorTopic = "/vrx/wayfinding/mean_error";

  /// \brief Publisher for the goal.
  // TODO: check transport::Node:Publisher 
  public: transport::Node::Publisher waypointsPub;

  /// \brief Publisher for the combined 2D pose error.
  public: transport::Node::Publisher minErrorsPub;

  /// \brief Publisher for the current rms error.
  public: transport::Node::Publisher meanErrorPub;

  /// \brief Vector containing waypoints as 3D vectors of doubles representing
  /// X Y yaw, where X and Y are local (Gazebo) coordinates.
  // TODO: Is this the right type?
  public: std::vector<math::Vector3d> localWaypoints;

  /// \brief Vector containing waypoints as 3D vectors of doubles representing
  /// Lattitude Longitude yaw, where lattitude and longitude are given in
  /// spherical (WGS84) coordinates.
  // TODO: Is this the right type?
  public: std::vector<math::Vector3d> sphericalWaypoints;

  // For publishing the waypoint locations
  public: msgs::Pose_V waypointsMessage;

  /// \brief Vector containing current minimum 2D pose error achieved for each
  /// waypoint so far.
  public: std::vector<double> minErrors;

  /// \brief Current average minimum error for all waypoints.
  public: double meanError;

  /// \brief Timer used to calculate the elapsed time docked in the bay.
  public: std::chrono::duration<double> timer;

  /// \brief Pointer to the SDF plugin element.
  public: sdf::ElementPtr sdf;

  /// \brief Spherical coordinate conversions. 
  public: math::SphericalCoordinates sc; 

  /// \brief Waypoint visualization markers.
  // TODO: get this working later
  // private: WaypointMarkers waypointMarkers;

  /// \brief Publish the waypoints through which the vehicle must navigate.
  // TODO: implement
  // public: void PublishWaypoints();
  
  public: gazebo::Entity vehicleEntity;

};
// TODO: change ignerr messages to more appropriate values
/////////////////////////////////////////////////
// WayfindingScoringPlugin::WayfindingScoringPlugin()
  // TODO: uncomment after fixing WaypointMarkers.hh 
//  : waypointMarkers("waypoint_marker")
WayfindingScoringPlugin::WayfindingScoringPlugin()
    : ScoringPlugin(), dataPtr(ignition::utils::MakeUniqueImpl<Implementation>())
{
  ignerr << "Wayfinding scoring plugin loaded" << std::endl;
  // TODO: uncomment to test timer 
//this->timer.Stop();
//this->timer.Reset();
}
WayfindingScoringPlugin::~WayfindingScoringPlugin()
{
}
void WayfindingScoringPlugin::Configure(const gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gazebo::EntityComponentManager &_ecm,
                           gazebo::EventManager &_eventMgr)
{
  ScoringPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  ignmsg << "Task [" << this->TaskName() << "]" << std::endl;

  this->dataPtr->meanError = 0.0; // TODO: delete
  this->dataPtr->sdf = _sdf->Clone();

  auto worldEntity = _ecm.EntityByComponents(gazebo::components::World());
  gazebo::World world(worldEntity);

  this->dataPtr->sc = world.SphericalCoordinates(_ecm).value();

  // A waypoints element is required.
  if (!this->dataPtr->sdf->HasElement("waypoints"))
  {
    ignerr << "Unable to find <waypoints> element in SDF." << std::endl;
    return;
  }
  auto waypointsElem = this->dataPtr->sdf->GetElement("waypoints");
 
  ignerr << "Found <waypoints> element in SDF." << std::endl;
  // We need at least one waypoint
  if (!waypointsElem->HasElement("waypoint"))
  {
    ignerr << "Unable to find <waypoint> element in <waypoints>." << std::endl;
    return;
  }
  auto waypointElem = waypointsElem->GetElement("waypoint");
 
  ignerr << "Found first <waypoint> element in <waypoints>." << std::endl;

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
    ignerr << "Waypoint, Spherical: Lat = " << latlonyaw.X()
          << " Lon = " << latlonyaw.Y() << std::endl;
    ignerr << "Waypoint, Local: X = " << cartVec.X()
          << " Y = " << cartVec.Y() << " Yaw = " << cartVec.Z() << std::endl;
  
    waypointElem = waypointElem->GetNextElement("waypoint");
  }

  // Throttle messages to 1Hz
  transport::AdvertiseMessageOptions opts;
  opts.SetMsgsPerSec(1u);
  if (this->dataPtr->sdf->HasElement("waypoints_topic"))
  {
      this->dataPtr->waypointsTopic = this->dataPtr->sdf->Get<std::string>("waypoints_topic");
  }
  this->dataPtr->waypointsPub =
      this->dataPtr->node.Advertise<msgs::Pose_V>(this->dataPtr->waypointsTopic, opts);

  if (_sdf->HasElement("min_errors_topic"))
  {
      this->dataPtr->minErrorsTopic = _sdf->Get<std::string>("min_errors_topic");
  }
  this->dataPtr->minErrorsPub = this->dataPtr->node.Advertise<msgs::Float_V>(this->dataPtr->minErrorsTopic, opts);

  if (_sdf->HasElement("mean_error_topic"))
  {
      this->dataPtr->meanErrorTopic = _sdf->Get<std::string>("mean_error_topic");
  }
  this->dataPtr->meanErrorPub = this->dataPtr->node.Advertise<msgs::Float>(
      this->dataPtr->meanErrorTopic, opts);

// // TODO: Publish waypoint markers
// if (_sdf->HasElement("markers"))
// {
//   this->waypointMarkers.Load(_sdf->GetElement("markers"));
//   if (this->waypointMarkers.IsAvailable())
//   {
//     int markerId = 0;
//     for (const auto waypoint : this->localWaypoints)
//     {
//       if (!this->waypointMarkers.DrawMarker(markerId, waypoint.X(),
//           waypoint.Y(), waypoint.Z(), std::to_string(markerId)))
//       {
//         gzerr << "Error creating visual marker" << std::endl;
//       }
//       markerId++;
//     }
//   }
//   else
//   {
//     gzwarn << "Cannot display gazebo markers (Gazebo version < 8)"
//            << std::endl;
//   }
// }

}

void WayfindingScoringPlugin::PreUpdate(const gazebo::UpdateInfo &_info,
                     gazebo::EntityComponentManager &_ecm)
{
  ScoringPlugin::PreUpdate(_info,_ecm);

  // Start publishing the goal once in "ready" state
  if (this->ScoringPlugin::TaskState() == "ready")
        this->dataPtr->waypointsPub.Publish(this->dataPtr->waypointsMessage);

  if (!this->dataPtr->vehicleEntity)
  {
      auto entity = _ecm.EntityByComponents(
          gazebo::components::Name(ScoringPlugin::VehicleName()));
      if (entity != gazebo::kNullEntity)
          this->dataPtr->vehicleEntity = entity;
      else
          return;
  }

   // Nothing to do if the task is not in "running" state.
  if (this->ScoringPlugin::TaskState() != "running")
    return;

  msgs::Float_V minErrorsMsg;
  msgs::Float meanErrorMsg;
  //ignerr << "Wayfinding: Running state" << std::endl;
  auto vehiclePose = _ecm.Component<gazebo::components::Pose>(
                             this->dataPtr->vehicleEntity)->Data();
  double currentHeading = vehiclePose.Rot().Euler().Z();
  double currentTotalError = 0;

  for (unsigned i = 0; i < this->dataPtr->localWaypoints.size(); ++i)
  {
    const math::Vector3d wp = this->dataPtr->localWaypoints[i];
    double dx   =  wp.X() - vehiclePose.Pos().X();
    double dy   =  wp.Y() - vehiclePose.Pos().Y();
    double dist = sqrt(pow(dx, 2) + pow(dy, 2));
    double k    = 0.75;
    double dhdg = abs(wp.Z() - currentHeading);
    double headError = M_PI - abs(dhdg - M_PI);

    double poseError =  dist + (pow(k, dist) * headError);

    // If this is the first time through, minError == poseError
    if (i == this->dataPtr->minErrors.size())
    {
      this->dataPtr->minErrors.push_back(poseError);
    }

    // If poseError is smaller than the minimum, update the minimum
    if (poseError < this->dataPtr->minErrors.at(i))
    {
      this->dataPtr->minErrors.at(i) = poseError;
    }

    // add current minimum to current total error
    currentTotalError += this->dataPtr->minErrors.at(i);
  }

  this->dataPtr->meanError = currentTotalError / this->dataPtr->localWaypoints.size();

  // set up messages
  meanErrorMsg.set_data(this->dataPtr->meanError);
  for (unsigned i = 0; i < this->dataPtr->minErrors.size(); ++i)
  {
    minErrorsMsg.add_data(this->dataPtr->minErrors.at(i));
  }

  // publish
  this->dataPtr->minErrorsPub.Publish(minErrorsMsg);
  this->dataPtr->meanErrorPub.Publish(meanErrorMsg);

  // START
  // TODO: Min errors not getting published
  ScoringPlugin::SetScore(this->dataPtr->meanError); 
}


//////////////////////////////////////////////////
// TODO: Start here and compare to PreUpdate 
// void WayfindingScoringPlugin::Update()
// {
//   // The vehicle might not be ready yet, let's try to get it.
//   if (!this->vehicleModel)
//   {
//     #if GAZEBO_MAJOR_VERSION >= 8
//       this->vehicleModel = this->world->ModelByName(this->vehicleName);
//     #else
//       this->vehicleModel = this->world->GetModel(this->vehicleName);
//     #endif
//     if (!this->vehicleModel)
//       return;
//   }
// 
//   // Nothing to do if the task is not in "running" state.
//   if (this->ScoringPlugin::TaskState() != "running")
//     return;
// 
//   std_msgs::Float64MultiArray minErrorsMsg;
//   std_msgs::Float64 meanErrorMsg;
// 
//   #if GAZEBO_MAJOR_VERSION >= 8
//     const auto robotPose = this->vehicleModel->WorldPose();
//   #else
//     const auto robotPose = this->vehicleModel->GetWorldPose().Ign();
//   #endif
//   double currentHeading = robotPose.Rot().Euler().Z();
// 
//   double currentTotalError = 0;
// 
//   for (unsigned i = 0; i < this->localWaypoints.size(); ++i)
//   {
//     const ignition::math::Vector3d wp = this->localWaypoints[i];
//     double dx   =  wp.X() - robotPose.Pos().X();
//     double dy   =  wp.Y() - robotPose.Pos().Y();
//     double dist = sqrt(pow(dx, 2) + pow(dy, 2));
//     double k    = 0.75;
//     double dhdg = abs(wp.Z() - currentHeading);
//     double headError = M_PI - abs(dhdg - M_PI);
// 
//     double poseError =  dist + (pow(k, dist) * headError);
// 
//     // If this is the first time through, minError == poseError
//     if (i == this->minErrors.size())
//     {
//       this->minErrors.push_back(poseError);
//     }
// 
//     // If poseError is smaller than the minimum, update the minimum
//     if (poseError < this->minErrors.at(i))
//     {
//       this->minErrors.at(i) = poseError;
//     }
// 
//     // add current minimum to current total error
//     currentTotalError += this->minErrors.at(i);
//   }
// 
//   this->meanError = currentTotalError / this->localWaypoints.size();
// 
//   // Set up multi array dimensions
//   minErrorsMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
//   minErrorsMsg.layout.dim[0].label = "minimum errors";
//   minErrorsMsg.layout.dim[0].size = this->localWaypoints.size();
//   minErrorsMsg.layout.dim[0].stride = this->localWaypoints.size();
// 
//   minErrorsMsg.data = this->minErrors;
//   meanErrorMsg.data = this->meanError;
// 
//   // Publish at 1 Hz.
//   if (this->timer.GetElapsed() >= gazebo::common::Time(1.0))
//   {
//     this->minErrorsPub.publish(minErrorsMsg);
//     this->meanErrorPub.publish(meanErrorMsg);
//     this->timer.Reset();
//     this->timer.Start();
//   }
// 
//   this->ScoringPlugin::SetScore(this->meanError);
// }

//////////////////////////////////////////////////
// void WayfindingScoringPlugin::PublishWaypoints()
// {
//   ignmsg << "<WayfindingScoringPlugin> Publishing Waypoints" << std::endl;
//   // moved to configure
//   this->waypointsPub.publish(path_msg);
// }

//////////////////////////////////////////////////
void WayfindingScoringPlugin::OnReady()
{
  ignerr << "WayfindingScoringPlugin::OnReady" << std::endl;
  // TODO: uncomment after implementing function
  // this->PublishWaypoints();
}


//////////////////////////////////////////////////
void WayfindingScoringPlugin::OnRunning()
{
  ignerr << "WayfindingScoringPlugin::OnRunning" << std::endl;
  // TODO: uncomment to test timer 
  // this->timer.Start();
}

IGNITION_ADD_PLUGIN(WayfindingScoringPlugin,
                    gazebo::System,
                    ScoringPlugin::ISystemConfigure,
                    ScoringPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(vrx::WayfindingScoringPlugin,
                          "vrx::WayfindingScoringPlugin")
// TODO: delete?
// Register plugin with gazebo
// GZ_REGISTER_WORLD_PLUGIN(WayfindingScoringPlugin)
