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

#ifndef VRX_GAZEBO_FOLLOW_PLUGIN_HH_
#define VRX_GAZEBO_FOLLOW_PLUGIN_HH_

#include <vector>
#include <cmath>
#include <gazebo/gazebo.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Pose3.hh>
#include <sdf/sdf.hh>
#include "vrx_gazebo/waypoint_markers.hh"

namespace gazebo
{
/// \brief A plugin that scripts the movement of a model based on waypoints.
///
/// This plugin loads a set of waypoints via SDF and traverse the waypoints
/// in sequence. The movement is generated applying force and torque to one of
/// of the model links. It's also possible to loop through the waypoints for
/// generating a never ending trajectory.
/// Waypoints may be inserted manually via the <waypoints> element, or
/// generated relative to the model's initial position via the <line> or
/// <circle> elements.  Only one of these options should be used.
///
/// This plugin requires the following SDF parameters:
/// * Required parameters:
/// <link_name>: The name of the link within the model where the force/torque
///              will be applied when moving the vehicle.
///
/// * Optional parameters:
/// <loop_forever>: When true, all waypoints will be visited continously in a
///                 circular pattern. If false, the model will stop when the
///                 last waypoint is reached. Note, that if the vehicle moves,
///                 it will still try to reach the very last waypoint.
/// <markers>: Enable visualization markers. Check the WaypointMarkers class for
///            SDF documentation.
/// <waypoints>: Element specifying the set of waypoints that the
///              the model should navigate through. This block should contain at
///              least one of these blocks:
///                <waypoint>: This block should contain the X, Y of a waypoint.
/// <line>: Element that specifies a direction and distance from the model's
///         initial position where a waypoint should be created.  The model's
///         initial position is also added as a waypoint.
///          <direction>:  Relative direction (radians) in the model's frame for
///                        second waypoint.
///          <length>:     Distance (meters) at which to set second waypoint.
/// <circle>: Element that indicates the model should travel in "circle" mode.
///           The block must contain the desired radius of the circle about the
///           vehicle's initial position.
///           <radius>:  Radius (meters) of circular path to travel.
///
/// Here are three examples:
/// <plugin name="CrocodileFollowPlugin" filename="libfollow_plugin.so">
///   <link_name>link</link_name>
///   <loop_forever>true</loop_forever>
///   <waypoints>
///     <waypoint>534 -172</waypoint>
///     <waypoint>532 -168</waypoint>
///   </waypoints>
///   <markers>
///     <material>Gazebo/Red</material>
///     <scaling>0.2 0.2 2.0</scaling>
///     <height>0.5</height>
///   </markers>
/// </plugin>
/// <plugin name='PlatypusFollowPlugin' filename='libfollow_plugin.so'>
///   <link_name>link</link_name>
///   <loop_forever>true</loop_forever>
///   <line>
///     <direction>1.5</direction>
///     <length>10</length>
///   </line>
/// </plugin>
/// <plugin name='TurtleFollowPlugin' filename='libfollow_plugin.so'>
///   <link_name>link</link_name>
///   <loop_forever>true</loop_forever>
///   <circle>
///      <radius>2</radius>
///   </circle>
/// </plugin>
class FollowPlugin : public ModelPlugin
{
  // \brief Constructor.
  public: FollowPlugin();

  // Documentation inherited.
  public: void Load(physics::ModelPtr _model,
                    sdf::ElementPtr _sdf);

  // Documentation inherited.
  private: virtual void Update();

  /// \brief Pointer to the model.
  private: physics::ModelPtr model;

  /// \brief Pointer to the model link.
  private: physics::LinkPtr link;

  /// \brief The initial pose of the model relative to the world frame.
  private: ignition::math::Pose3d modelPose;

  /// \brief True if the model should continue looping though the waypoints.
  private: bool loopForever = false;

  /// \brief Linear force to apply to the model in its X direction.
  private: double forceToApply = 10;

  /// \brief Torque to apply to the model to align it with the next goal.
  private: double torqueToApply = 1;

  /// \brief When the model is at this distance or closer we won't try to move.
  /// Units are in meters.
  private: double rangeGoal = 0.5;

  /// \brief When the model is at this angle or closer we won't try to rotate.
  /// Units are in degrees.
  private: double bearingGoal = 2.0;

  /// \brief The next position to reach.
  private: ignition::math::Vector3d nextGoal;

  /// \brief Vector containing waypoints as 3D vectors of doubles representing
  /// X Y, where X and Y are local (Gazebo) coordinates.
  private: std::vector<ignition::math::Vector2d> localWaypoints;

  /// \brief Waypoint visualization markers.
  private: WaypointMarkers waypointMarkers;

  /// \brief Pointer used to connect gazebo callback to plugins update function.
  private: event::ConnectionPtr updateConnection;
};
}
#endif
