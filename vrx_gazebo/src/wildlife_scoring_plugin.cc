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

#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPath.h>
#include <algorithm>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/physics/Link.hh>
#include <ignition/math/Helpers.hh>
#include "vrx_gazebo/wildlife_scoring_plugin.hh"

/////////////////////////////////////////////////
WildlifeScoringPlugin::VirtualGate::VirtualGate(
    const gazebo::physics::LinkPtr _leftMakerLink,
    const ignition::math::Vector3d &_offset,
    double _width)
  : leftMarkerLink(_leftMakerLink),
    offset(_offset),
    width(_width)
{
  this->Update();
}

/////////////////////////////////////////////////
void WildlifeScoringPlugin::VirtualGate::Update()
{
  if (!this->leftMarkerLink)
    return;

  // The pose of the markers delimiting the virtual gate.
#if GAZEBO_MAJOR_VERSION >= 8
  const auto leftMarkerPos = this->leftMarkerLink->WorldPose().Pos();
#else
  const auto leftMarkerPos = this->leftMarkerLink->GetWorldPose().Ign().Pos();
#endif
  const auto rightMarkerPos = leftMarkerPos + this->offset;

  // Unit vector from the left marker to the right one.
  auto v1 = leftMarkerPos - rightMarkerPos;
  v1.Normalize();

  // Unit vector perpendicular to v1 in the direction we like to cross gates.
  const auto v2 = -ignition::math::Vector3d::UnitZ.Cross(v1);

  // This is the center point of the gate.
  const auto middle = (leftMarkerPos + rightMarkerPos) / 2.0;

  // Yaw of the gate in world coordinates.
  const auto yaw = atan2(v2.Y(), v2.X());

  // The updated pose.
  this->pose.Set(middle, ignition::math::Vector3d(0, 0, yaw));
}

/////////////////////////////////////////////////
WildlifeScoringPlugin::GateState
  WildlifeScoringPlugin::VirtualGate::IsPoseInGate(
    const ignition::math::Pose3d &_robotWorldPose) const
{
  // Transform to gate frame.
  const ignition::math::Vector3d robotLocalPosition =
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

/////////////////////////////////////////////////
WildlifeScoringPlugin::Buoy::Buoy(
    const gazebo::physics::LinkPtr _buoyLink,
    const BuoyGoal _buoyGoal,
    double _engagementDistance)
  : link(_buoyLink),
    goal(_buoyGoal),
    engagementDistance(_engagementDistance)
{
  if (this->goal == BuoyGoal::AVOID)
    return;

  // Initialize the virtual gates.
  const double kAangleIncrement = 2 * IGN_PI / this->kNumVirtualGates;
  for (int i = 0; i < this->kNumVirtualGates; ++i)
  {
    double alpha = kAangleIncrement * i;
    ignition::math::Vector3d offset;
    offset.X(this->engagementDistance * cos(alpha));
    offset.Y(this->engagementDistance * sin(alpha));
#if GAZEBO_MAJOR_VERSION >= 8
    offset.Z(this->link->WorldPose().Pos().Z());
#else
    offset.Z(this->link->GetWorldPose().Ign().Pos().Z());
#endif

    this->virtualGates.push_back(
      VirtualGate(_buoyLink, offset, _engagementDistance));
  }

  this->Update();
}

/////////////////////////////////////////////////
void WildlifeScoringPlugin::Buoy::Update()
{
  if (this->state == BuoyState::CIRCUMNAVIGATED || !this->vehicleModel)
    return;

#if GAZEBO_MAJOR_VERSION >= 8
  const ignition::math::Pose3d vehiclePose = this->vehicleModel->WorldPose();
  const ignition::math::Pose3d buoyPose = this->link->WorldPose();
#else
  const ignition::math::Pose3d vehiclePose =
    this->vehicleModel->GetWorldPose().Ign();
  const ignition::math::Pose3d buoyPose = this->link->GetWorldPose().Ign();
#endif
  const double vehicleBuoyDistance = vehiclePose.Pos().Distance(buoyPose.Pos());

  if (this->state == BuoyState::NEVER_ENGAGED)
  {
    if (vehicleBuoyDistance <= this->engagementDistance)
    {
      // Transition to ENGAGED.
      this->state = BuoyState::ENGAGED;
      gzdbg << "[WildlifeScoringPlugin::Buoy] " << this->link->GetScopedName()
            << " Transition from NEVER_ENGAGED" << " to ENGAGED" << std::endl;
      return;
    }
  }
  else if (this->state == BuoyState::NOT_ENGAGED)
  {
    if (vehicleBuoyDistance <= this->engagementDistance)
    {
      // Transition to ENGAGED.
      this->state = BuoyState::ENGAGED;
      gzdbg << "[WildlifeScoringPlugin::Buoy] " << this->link->GetScopedName()
            << " Transition from NOT_ENGAGED" << " to ENGAGED" << std::endl;
      return;
    }
  }
  else if (this->state == BuoyState::ENGAGED)
  {
    if (vehicleBuoyDistance > this->engagementDistance)
    {
      // Transition to NOT ENGAGED.
      this->state = BuoyState::NOT_ENGAGED;
      gzdbg << "[WildlifeScoringPlugin::Buoy] " << this->link->GetScopedName()
            << " Transition from ENGAGED" << " to NOT_ENGAGED" << std::endl;

      // You need to start over.
      this->numVirtualGatesCrossed = 0u;
      return;
    }

    if (this->goal == BuoyGoal::AVOID)
      return;

    // Check circumnavigation using the virtual gates.
    for (auto &virtualGate : this->virtualGates)
    {
      virtualGate.Update();

      // Check if we have crossed this gate.
      auto currentState = virtualGate.IsPoseInGate(vehiclePose);
      if (currentState == GateState::VEHICLE_AFTER &&
          virtualGate.state == GateState::VEHICLE_BEFORE)
      {
        currentState = GateState::CROSSED;

        if (this->goal == BuoyGoal::CIRCUMNAVIGATE_COUNTERCLOCKWISE)
        {
          ++this->numVirtualGatesCrossed;
          gzdbg << "[WildlifeScoringPlugin::Buoy] "
                << this->link->GetScopedName()
                << " Virtual gate crossed counterclockwise! ("
                << 100 * this->numVirtualGatesCrossed / this->kNumVirtualGates
                << "% completed)" << std::endl;
        }
        else
        {
          this->numVirtualGatesCrossed = 0u;
          gzdbg << "[WildlifeScoringPlugin::Buoy] "
                << this->link->GetScopedName()
                << " Virtual gate incorrectly crossed counterclockwise! ("
                << 100 * this->numVirtualGatesCrossed / this->kNumVirtualGates
                << "% completed)" << std::endl;
        }
      }
      else if (currentState == GateState::VEHICLE_BEFORE &&
               virtualGate.state == GateState::VEHICLE_AFTER)
      {
        currentState = GateState::CROSSED;

        if (this->goal == BuoyGoal::CIRCUMNAVIGATE_CLOCKWISE)
        {
          ++this->numVirtualGatesCrossed;
          gzdbg << "[WildlifeScoringPlugin::Buoy] "
                << this->link->GetScopedName()
                << " Virtual gate crossed clockwise! ("
                << 100 * this->numVirtualGatesCrossed / this->kNumVirtualGates
                << "% completed)" << std::endl;
        }
        else
        {
          this->numVirtualGatesCrossed = 0u;
          gzdbg << "[WildlifeScoringPlugin::Buoy] "
                << this->link->GetScopedName()
                << " Virtual gate incorrectly crossed clockwise! ("
                << 100 * this->numVirtualGatesCrossed / this->kNumVirtualGates
                << "% completed)" << std::endl;
        }
      }

      if (this->numVirtualGatesCrossed == this->kNumVirtualGates)
        this->state = BuoyState::CIRCUMNAVIGATED;

      virtualGate.state = currentState;
    }
  }
}

/////////////////////////////////////////////////
void WildlifeScoringPlugin::Buoy::SetVehicleModel(
  gazebo::physics::ModelPtr _vehicleModel)
{
  this->vehicleModel = _vehicleModel;
}

/////////////////////////////////////////////////
WildlifeScoringPlugin::WildlifeScoringPlugin()
{
  gzmsg << "Wildlife scoring plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void WildlifeScoringPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  ScoringPlugin::Load(_world, _sdf);

  // Parse the optional <animals_topic> element.
  if (_sdf->HasElement("animals_topic"))
    this->animalsTopic = _sdf->Get<std::string>("animals_topic");

  // Parse the optional <engagement_distance> element.
  if (_sdf->HasElement("engagement_distance"))
    this->engagementDistance = _sdf->Get<double>("engagement_distance");

  // Parse the optional <time_bonus> element.
  if (_sdf->HasElement("time_bonus"))
    this->timeBonus = _sdf->Get<double>("time_bonus");

  // Parse the optional <buoys> element.
  // Note: Parse this element at the end because we use some of the previous
  // parameters.
  if (_sdf->HasElement("buoys"))
  {
    auto const &buoysElem = _sdf->GetElement("buoys");
    if (!this->ParseBuoys(buoysElem))
    {
      gzerr << "Score has been disabled" << std::endl;
      return;
    }
  }

  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  // Setup ROS node and publisher
  this->rosNode.reset(new ros::NodeHandle());

  this->animalsPub =
    this->rosNode->advertise<geographic_msgs::GeoPath>(
      this->animalsTopic, 10, true);

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&WildlifeScoringPlugin::Update, this));
}

//////////////////////////////////////////////////
bool WildlifeScoringPlugin::ParseBuoys(sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_sdf, "WildlifeScoringPlugin::ParseBuoys(): NULL _sdf pointer");

  // We need at least one buoy.
  if (!_sdf->HasElement("buoy"))
  {
    gzerr << "Unable to find <buoy> element in SDF." << std::endl;
    return false;
  }

  auto buoyElem = _sdf->GetElement("buoy");

  // Parse a new buoy.
  while (buoyElem)
  {
    // The buoy's model.
    if (!buoyElem->HasElement("model_name"))
    {
      gzerr << "Unable to find <buoys><buoy><model_name> element in SDF."
            << std::endl;
      return false;
    }

    const std::string buoyModelName = buoyElem->Get<std::string>("model_name");

    // The buoy's main link.
    if (!buoyElem->HasElement("link_name"))
    {
      gzerr << "Unable to find <buoys><buoy><link_name> element in SDF."
            << std::endl;
      return false;
    }

    const std::string buoyLinkName = buoyElem->Get<std::string>("link_name");

    // The buoy's goal.
    if (!buoyElem->HasElement("goal"))
    {
      gzerr << "Unable to find <buoys><buoy><goal> element in SDF."
            << std::endl;
      return false;
    }

    const std::string buoyGoal = buoyElem->Get<std::string>("goal");
    if (!this->AddBuoy(buoyModelName, buoyLinkName, buoyGoal))
      return false;

    // Parse the next buoy.
    buoyElem = buoyElem->GetNextElement("buoy");
  }

  return true;
}

//////////////////////////////////////////////////
bool WildlifeScoringPlugin::AddBuoy(const std::string &_modelName,
    const std::string &_linkName, const std::string &_goal)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::ModelPtr parentModel = this->world->ModelByName(_modelName);
#else
  gazebo::physics::ModelPtr parentModel = this->world->GetModel(_modelName);
#endif
  // Sanity check: Make sure that the model exists.
  if (!parentModel)
  {
    gzerr << "Unable to find model [" << _modelName << "]" << std::endl;
    return false;
  }

  gazebo::physics::LinkPtr link = parentModel->GetLink(_linkName);
  // Sanity check: Make sure that the link exists.
  if (!link)
  {
    gzerr << "Unable to find link [" << _linkName << "]" << std::endl;
    return false;
  }

  BuoyGoal buoyGoal;
  if (_goal == "avoid")
    buoyGoal = BuoyGoal::AVOID;
  else if (_goal == "circumnavigate_clockwise")
    buoyGoal = BuoyGoal::CIRCUMNAVIGATE_CLOCKWISE;
  else if (_goal == "circumnavigate_counterclockwise")
    buoyGoal = BuoyGoal::CIRCUMNAVIGATE_COUNTERCLOCKWISE;
  else
  {
    gzerr << "Unknown <goal> value: [" << _goal << "]" << std::endl;
    return false;
  }

  // Save the new buoy.
  this->buoys.push_back(Buoy(link, buoyGoal, this->engagementDistance));

  return true;
}

//////////////////////////////////////////////////
void WildlifeScoringPlugin::Update()
{
  // The vehicle is not in the simulation yet.
  if (!this->vehicleModel)
    return;

  // Skip if we're not in running mode.
  if (this->TaskState() != "running")
    return;

  // Current score.
  this->ScoringPlugin::SetScore(
    std::min(this->GetRunningStateDuration(), this->ElapsedTime().Double()));

  // Update the state of all buoys.
  bool taskCompleted = true;
  for (auto &buoy : this->buoys)
  {
    // Update the vehicle model if needed.
    if (!buoy.vehicleModel)
      buoy.SetVehicleModel(this->vehicleModel);

    // If a collision is detected, invalidate the circumnavigations.
    if (this->collisionDetected)
      buoy.numVirtualGatesCrossed = 0u;

    // Update the buoy state.
    buoy.Update();

    // We consider the task completed when all the circumnavigation goals have
    // been completed.
    if ((buoy.goal == BuoyGoal::CIRCUMNAVIGATE_CLOCKWISE ||
         buoy.goal == BuoyGoal::CIRCUMNAVIGATE_COUNTERCLOCKWISE) &&
        buoy.state != BuoyState::CIRCUMNAVIGATED)
    {
      taskCompleted = false;
    }
  }

  this->collisionDetected = false;

  // Publish the location of the buoys.
  this->PublishAnimalLocations();

  if (taskCompleted)
  {
    gzmsg << "Course completed!" << std::endl;
    this->SetScore(std::max(0.0, this->Score() - this->TimeBonus()));
    this->Finish();
  }
}

//////////////////////////////////////////////////
void WildlifeScoringPlugin::PublishAnimalLocations()
{
  geographic_msgs::GeoPath geoPathMsg;
  geoPathMsg.header.stamp = ros::Time::now();

  for (auto const &buoy : this->buoys)
  {
    // Conversion from Gazebo Cartesian coordinates to spherical.
#if GAZEBO_MAJOR_VERSION >= 8
    const ignition::math::Pose3d pose = buoy.link->WorldPose();
#else
    const ignition::math::Pose3d pose = buoy.link->GetWorldPose().Ign();
#endif

    auto in = ignition::math::SphericalCoordinates::CoordinateType::GLOBAL;
    auto out = ignition::math::SphericalCoordinates::CoordinateType::SPHERICAL;
    auto latlon = this->sc.PositionTransform(pose.Pos(), in, out);
    latlon.X(IGN_RTOD(latlon.X()));
    latlon.Y(IGN_RTOD(latlon.Y()));

    const ignition::math::Quaternion<double> orientation = pose.Rot();

    // Fill the GeoPoseStamped message.
    geographic_msgs::GeoPoseStamped geoPoseMsg;
    geoPoseMsg.header.stamp = ros::Time::now();

    // We set the buoy type based on its goal.
    if (buoy.goal == BuoyGoal::AVOID)
      geoPoseMsg.header.frame_id = "crocodile";
    else if (buoy.goal == BuoyGoal::CIRCUMNAVIGATE_CLOCKWISE)
      geoPoseMsg.header.frame_id = "platypus";
    else if (buoy.goal == BuoyGoal::CIRCUMNAVIGATE_COUNTERCLOCKWISE)
      geoPoseMsg.header.frame_id = "turtle";
    else
      geoPoseMsg.header.frame_id = "unknown";

    geoPoseMsg.pose.position.latitude  = latlon.X();
    geoPoseMsg.pose.position.longitude = latlon.Y();
    geoPoseMsg.pose.position.altitude  = latlon.Z();
    geoPoseMsg.pose.orientation.x = orientation.X();
    geoPoseMsg.pose.orientation.y = orientation.Y();
    geoPoseMsg.pose.orientation.z = orientation.Z();
    geoPoseMsg.pose.orientation.w = orientation.W();

    // Add the GeoPoseStamped message to the GeoPath message that we publish.
    geoPathMsg.poses.push_back(geoPoseMsg);
  }
  this->animalsPub.publish(geoPathMsg);
}

//////////////////////////////////////////////////
void WildlifeScoringPlugin::OnCollision()
{
  gzdbg << "Collision detected, invalidating circumnavigations" << std::endl;
  this->collisionDetected = true;
}

//////////////////////////////////////////////////
double WildlifeScoringPlugin::TimeBonus() const
{
  // Check time bonuses.
  double totalBonus = 0;
  for (auto const &buoy : this->buoys)
  {
    if (buoy.goal == BuoyGoal::AVOID &&
        buoy.state == BuoyState::NEVER_ENGAGED)
    {
      totalBonus += this->timeBonus;
    }
    else if (buoy.goal == BuoyGoal::CIRCUMNAVIGATE_CLOCKWISE &&
             buoy.state == BuoyState::CIRCUMNAVIGATED)
    {
      totalBonus += this->timeBonus;
    }
    else if (buoy.goal == BuoyGoal::CIRCUMNAVIGATE_COUNTERCLOCKWISE &&
             buoy.state == BuoyState::CIRCUMNAVIGATED)
    {
      totalBonus += this->timeBonus;
    }
  }

  return totalBonus;
}

//////////////////////////////////////////////////
void WildlifeScoringPlugin::OnFinished()
{
  this->SetTimeoutScore(std::max(0.0, this->Score() - this->TimeBonus()));
  ScoringPlugin::OnFinished();
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(WildlifeScoringPlugin)
