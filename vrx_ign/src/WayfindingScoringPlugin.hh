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

#ifndef VRX_WAYFINDINGSCORINGPLUGIN_HH_
#define VRX_WAYFINDINGSCORINGPLUGIN_HH_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/utils/ImplPtr.hh>
#include <ignition/transport/Node.hh>
// #include <gazebo/common/Events.hh>
// #include <gazebo/common/Timer.hh>
// #include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>

#include "ScoringPlugin.hh"
// #include "WaypointMarkers.hh"
namespace vrx
{
/// \brief A plugin for computing the score of the wayfinding navigation task.
/// This plugin derives from the generic ScoringPlugin class. Refer to that
/// plugin for an explanation of the four states defined (Initial, Ready,
/// Running and Finished) as well as other required SDF elements.
///
/// This plugin publishes a series of poses to a topic when it enters the Ready
/// states.
///
/// In the running state it calculates a 2D pose error distance between the
/// vehicle and each of the waypoints and stores the minimum error distance
/// achieved for each waypoint so far. The current recorded minimums are
/// published to a topic as an array of doubles. The average of all minimums is
/// also updated. This value is published to a separate topic for mean error,
/// and set as the current score using the SetScore() method inherited from
/// the parent. This causes it to also appear in the task information topic.
///
/// This plugin requires the following SDF parameters:
///
/// <waypoints>: Required element specifying the set of waypoints that the
/// the vehicle should navigate through.
///
///    This block should contain at least one of these blocks:
///     <waypoint>: This block should contain a <pose> element specifying the
///     lattitude, longitude and yaw of a waypoint.
/// <markers>: Optional parameter to enable visualization markers. Check the
/// WaypointMarkers class for SDF documentation.
class WayfindingScoringPlugin : public ScoringPlugin
{
  /// \brief Constructor.
  public: WayfindingScoringPlugin();
 
  // Destructor
  public: ~WayfindingScoringPlugin() override;

  // Documentation inherited.
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited. 
  /// \brief Callback executed at every world update.
    // TODO: Needed?
    public: void PreUpdate(
                      const ignition::gazebo::UpdateInfo &_info,
                      ignition::gazebo::EntityComponentManager &_ecm) override;


  // Documentation inherited.
  private: void OnReady() override;

  // Documentation inherited.
  private: void OnRunning() override;

  /// \brief Publish the waypoints through which the vehicle must navigate.
  // TODO: 
  // private: void PublishWaypoints();

  /// \brief Pointer to the update event connection.
  // TODO
  // private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief Pointer to the sdf plugin element.
  // TODO: Move to implementation, public?
  private: sdf::ElementPtr sdf;

  /// \brief Topic where the list of waypoints is published.
  // OK
  private: std::string waypointsTopic = "/vrx/wayfinding/waypoints";

  /// \brief Topic where the current minimum pose error distance for each
  /// waypoint is published.
  // OK
  private: std::string minErrorsTopic = "/vrx/wayfinding/min_errors";

  /// \brief Topic where the current average minimum error is published.
  // OK
  private: std::string meanErrorTopic = "/vrx/wayfinding/mean_error";

  /// \brief ROS node handle.
  // TODO: see transport node?
  // private: std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief Publisher for the goal.
  // TODO: check transport::Node:Publisher 
  // private: ignition::transport::Node:Publisher waypointsPub;

  /// \brief Publisher for the combined 2D pose error.
  // private: ignition::transport::Node:Publisher minErrorsPub;

  /// \brief Publisher for the current rms error.
  // private: ignition::transport::Node:Publisher meanErrorPub;

  /// \brief Vector containing waypoints as 3D vectors of doubles representing
  /// X Y yaw, where X and Y are local (Gazebo) coordinates.
  // TODO: Is this the right type?
  // TODO: Move to implementation, public?
  // private: std::vector<ignition::math::Vector3d> localWaypoints;

  /// \brief Vector containing waypoints as 3D vectors of doubles representing
  /// Lattitude Longitude yaw, where lattitude and longitude are given in
  /// spherical (WGS84) coordinates.
  // TODO: Is this the right type?
  // TODO: Move to implementation, public?
  // private: std::vector<ignition::math::Vector3d> sphericalWaypoints;

  /// \brief Vector containing current minimum 2D pose error achieved for each
  /// waypoint so far.
  // TODO: Move to implementation, public?
  private: std::vector<double> minErrors;

  /// \brief Current average minimum error for all waypoints.
  // TODO: Move to implementation, public?
  private: double meanError;

  /// \brief Timer used to calculate the elapsed time docked in the bay.
  // private: gazebo::common::Timer timer;

  /// \brief Waypoint visualization markers.
  // TODO: get this working later
  // private: WaypointMarkers waypointMarkers;
};

} // namespace
#endif
