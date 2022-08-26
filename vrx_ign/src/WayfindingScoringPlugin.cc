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

  /// \brief Vector containing current minimum 2D pose error achieved for each
  /// waypoint so far.
  public: std::vector<double> minErrors;

  /// \brief Current average minimum error for all waypoints.
  public: double meanError;

  /// \brief Timer used to calculate the elapsed time docked in the bay.
  public: std::chrono::duration<double> timer;

  /// \brief Waypoint visualization markers.
  // TODO: get this working later
  // private: WaypointMarkers waypointMarkers;

  /// \brief Publish the waypoints through which the vehicle must navigate.
  // TODO: implement
  // public: void PublishWaypoints();

};
/////////////////////////////////////////////////
// WayfindingScoringPlugin::WayfindingScoringPlugin()
  // TODO: uncomment after fixing WaypointMarkers.hh 
//  : waypointMarkers("waypoint_marker")
WayfindingScoringPlugin::WayfindingScoringPlugin()
    : ScoringPlugin(), dataPtr(utils::MakeUniqueImpl<Implementation>())
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
  this->dataPtr->meanError = 0.0;

  ignerr << "Task [" << this->TaskName() << "]" << std::endl;

  ignerr << "Wayfinding scoring plugin configured" << std::endl;
}

/////////////////////////////////////////////////
// TODO: Change to Configure
// void WayfindingScoringPlugin::Load(gazebo::physics::WorldPtr _world,
//     sdf::ElementPtr _sdf)
// {
//   ScoringPlugin::Load(_world, _sdf);
//   this->sdf = _sdf;
//   gzmsg << "Task [" << this->TaskName() << "]" << std::endl;
// 
//   // A waypoints element is required.
//   if (!this->sdf->HasElement("waypoints"))
//   {
//     gzerr << "Unable to find <waypoints> element in SDF." << std::endl;
//     return;
//   }
//   auto waypointsElem = this->sdf->GetElement("waypoints");
// 
//   // We need at least one waypoint
//   if (!waypointsElem->HasElement("waypoint"))
//   {
//     gzerr << "Unable to find <waypoint> element in SDF." << std::endl;
//     return;
//   }
//   auto waypointElem = waypointsElem->GetElement("waypoint");
// 
//   while (waypointElem)
//   {
//     ignition::math::Vector3d latlonyaw =
//       waypointElem->Get<ignition::math::Vector3d>("pose");
// 
//     // Convert lat/lon to local
//     //  snippet from UUV Simulator SphericalCoordinatesROSInterfacePlugin.cc
//     ignition::math::Vector3d scVec(latlonyaw.X(), latlonyaw.Y(), 0.0);
// 
//     ignition::math::Vector3d cartVec =
//       this->sc.LocalFromSphericalPosition(scVec);
// 
//     cartVec.Z() = latlonyaw.Z();
// 
//     // Set up relevant vectors
//     this->sphericalWaypoints.push_back(latlonyaw);
//     this->localWaypoints.push_back(cartVec);
// 
//     // Print some debugging messages
//     gzmsg << "Waypoint, Spherical: Lat = " << latlonyaw.X()
//           << " Lon = " << latlonyaw.Y() << std::endl;
//     gzmsg << "Waypoint, Local: X = " << cartVec.X()
//           << " Y = " << cartVec.Y() << " Yaw = " << cartVec.Z() << std::endl;
// 
//     waypointElem = waypointElem->GetNextElement("waypoint");
//   }
// 
//   // Setup ROS node and publisher
//   this->rosNode.reset(new ros::NodeHandle());
//   if (_sdf->HasElement("waypoints_topic"))
//   {
//     this->waypointsTopic = _sdf->Get<std::string>("waypoints_topic");
//   }
//   this->waypointsPub =
//     this->rosNode->advertise<geographic_msgs::GeoPath>(
//       this->waypointsTopic, 10, true);
// 
//   if (_sdf->HasElement("min_errors_topic"))
//   {
//     this->minErrorsTopic = _sdf->Get<std::string>("min_errors_topic");
//   }
//   this->minErrorsPub =
//     this->rosNode->advertise<std_msgs::Float64MultiArray>(
//       this->minErrorsTopic, 100);
// 
//   if (_sdf->HasElement("mean_error_topic"))
//   {
//     this->meanErrorTopic = _sdf->Get<std::string>("mean_error_topic");
//   }
//   this->meanErrorPub =
//     this->rosNode->advertise<std_msgs::Float64>(
//       this->meanErrorTopic, 100);
// 
//   this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
//     std::bind(&WayfindingScoringPlugin::Update, this));
// 
//   // Publish waypoint markers
//   if (_sdf->HasElement("markers"))
//   {
//     this->waypointMarkers.Load(_sdf->GetElement("markers"));
//     if (this->waypointMarkers.IsAvailable())
//     {
//       int markerId = 0;
//       for (const auto waypoint : this->localWaypoints)
//       {
//         if (!this->waypointMarkers.DrawMarker(markerId, waypoint.X(),
//             waypoint.Y(), waypoint.Z(), std::to_string(markerId)))
//         {
//           gzerr << "Error creating visual marker" << std::endl;
//         }
//         markerId++;
//       }
//     }
//     else
//     {
//       gzwarn << "Cannot display gazebo markers (Gazebo version < 8)"
//              << std::endl;
//     }
//   }
// }

void WayfindingScoringPlugin::PreUpdate(const gazebo::UpdateInfo &_info,
                     gazebo::EntityComponentManager &_ecm)
{
  if (this->dataPtr->meanError == 0.0) {
    ignerr << "Wayfinding scoring plugin preupdate" << std::endl;
    this->dataPtr->meanError++;
  }
   // The vehicle might not be ready yet, let's try to get it.
// if (!this->vehicleModel)
// {
//   #if GAZEBO_MAJOR_VERSION >= 8
//     this->vehicleModel = this->world->ModelByName(this->vehicleName);
//   #else
//     this->vehicleModel = this->world->GetModel(this->vehicleName);
//   #endif
//   if (!this->vehicleModel)
//     return;
// }
// ignerr << "Got the vehicle" << std::endl;
 
   // Nothing to do if the task is not in "running" state.
   if (this->ScoringPlugin::TaskState() != "running")
     return;

   ignerr << "Wayfinding: Running state" << std::endl;

 
}


//////////////////////////////////////////////////
// TODO: Change to PreUpdate
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
// TODO:
// void WayfindingScoringPlugin::PublishWaypoints()
// {
//   gzmsg << "<WayfindingScoringPlugin> Publishing Waypoints" << std::endl;
//   geographic_msgs::GeoPoseStamped wp_msg;
//   geographic_msgs::GeoPath path_msg;
// 
//   path_msg.header.stamp = ros::Time::now();
// 
//   for (auto wp : this->sphericalWaypoints)
//   {
//     wp_msg.pose.position.latitude  = wp.X();
//     wp_msg.pose.position.longitude = wp.Y();
//     wp_msg.pose.position.altitude  = 0.0;
// 
//     const ignition::math::Quaternion<double> orientation(0.0, 0.0, wp.Z());
// 
//     wp_msg.pose.orientation.x = orientation.X();
//     wp_msg.pose.orientation.y = orientation.Y();
//     wp_msg.pose.orientation.z = orientation.Z();
//     wp_msg.pose.orientation.w = orientation.W();
// 
//     wp_msg.header.stamp = ros::Time::now();
//     path_msg.poses.push_back(wp_msg);
//   }
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
