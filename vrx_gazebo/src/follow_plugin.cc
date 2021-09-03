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

#include <string>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include "vrx_gazebo/follow_plugin.hh"

using namespace gazebo;

/////////////////////////////////////////////////
FollowPlugin::FollowPlugin()
  : waypointMarkers("waypoint_marker")
{
}

//////////////////////////////////////////////////
void FollowPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != nullptr, "Received NULL model pointer");
  this->model = _model;

  // Parse the optional <waypoints> element.
  if (_sdf->HasElement("waypoints"))
  {
    auto waypointsElem = _sdf->GetElement("waypoints");

    // We need at least one waypoint
    if (!waypointsElem->HasElement("waypoint"))
    {
      gzerr << "FollowPlugin: Unable to find <waypoints><waypoint> element "
            << "in SDF." << std::endl;
      return;
    }
    auto waypointElem = waypointsElem->GetElement("waypoint");
    while (waypointElem)
    {
      ignition::math::Vector2d position =
        waypointElem->Get<ignition::math::Vector2d>();

      // Save the position.
      this->localWaypoints.push_back(position);

      // Print some debugging messages
      gzmsg << "Waypoint, Local: X = " << position.X()
            << " Y = " << position.Y() << std::endl;

      waypointElem = waypointElem->GetNextElement("waypoint");
    }
  }

  // Parse the required <link_name> element.
  if (!_sdf->HasElement("link_name"))
  {
    gzerr << "FollowPlugin: Missing <link_name> element\n";
    return;
  }

  std::string linkName = _sdf->GetElement("link_name")->Get<std::string>();
  this->link = this->model->GetLink(linkName);
  if (!this->link)
  {
    gzerr << "FollowPlugin: The link '" << linkName
          << "' does not exist within the model'" << std::endl;
    return;
  }

  // Parse the optional <loop_forever> element.
  if (_sdf->HasElement("loop_forever"))
    this->loopForever = true;

  // Parse the optional <markers> element.
  if (_sdf->HasElement("markers"))
  {
    this->waypointMarkers.Load(_sdf->GetElement("markers"));
    if (this->waypointMarkers.IsAvailable())
    {
      int markerId = 0;
      for (const auto waypoint : this->localWaypoints)
      {
        if (!this->waypointMarkers.DrawMarker(waypoint.X(),
            waypoint.Y(), 0, std::to_string(markerId)))
        {
          gzerr << "Error creating visual marker" << std::endl;
        }
        ++markerId;
      }
    }
    else
    {
      gzwarn << "Cannot display gazebo markers (Gazebo version < 8)"
             << std::endl;
    }
  }

  // If we have waypoints to visit, read the first one.
  if (!this->localWaypoints.empty())
  {
    this->nextGoal =
      {this->localWaypoints.front().X(), this->localWaypoints.front().Y(), 0};
  }

  // Connect the update function to the world update event.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&FollowPlugin::Update, this));
}

//////////////////////////////////////////////////
void FollowPlugin::Update()
{
  // Nothing to do.
  if (this->localWaypoints.empty())
    return;

  // Direction vector to the goal from the model.
  ignition::math::Vector3d direction =
#if GAZEBO_MAJOR_VERSION >= 8
    this->nextGoal - this->model->WorldPose().Pos();
#else
    this->nextGoal - this->model->GetWorldPose().Ign().Pos();
#endif

  // Direction vector in the local frame of the model.
  ignition::math::Vector3d directionLocalFrame =
#if GAZEBO_MAJOR_VERSION >= 8
    this->model->WorldPose().Rot().RotateVectorReverse(direction);
#else
    this->model->GetWorldPose().Ign().Rot().RotateVectorReverse(direction);
#endif

  double range = directionLocalFrame.Length();
  ignition::math::Angle bearing(
    atan2(directionLocalFrame.Y(), directionLocalFrame.X()));
  bearing.Normalize();

  // Waypoint reached!
  if (range <= this->rangeGoal)
  {
    // We always keep the last waypoint in the vector to keep the model
    // "alive" in case it moves away from its goal.
    if (this->localWaypoints.size() == 1)
      return;

    if (this->loopForever)
    {
      // Rotate to the left.
      std::rotate(this->localWaypoints.begin(),
                  this->localWaypoints.begin() + 1,
                  localWaypoints.end());
    }
    else
    {
      // Remove the first waypoint.
      this->localWaypoints.erase(this->localWaypoints.begin());
    }

    this->nextGoal =
      {this->localWaypoints.front().X(), this->localWaypoints.front().Y(), 0};

    return;
  }

  // Move commands. The vehicle always move forward (X direction).
  this->link->AddLinkForce({this->forceToApply, 0, 0});

  if (bearing.Degree() > this->bearingGoal)
    this->link->AddRelativeTorque({0, 0, this->torqueToApply});

  if (bearing.Degree() < -this->bearingGoal)
    this->link->AddRelativeTorque({0, 0, -this->torqueToApply});
}

GZ_REGISTER_MODEL_PLUGIN(FollowPlugin);
