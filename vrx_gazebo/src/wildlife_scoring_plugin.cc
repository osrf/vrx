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
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/physics/Link.hh>
#include "vrx_gazebo/wildlife_scoring_plugin.hh"

/////////////////////////////////////////////////
WildlifeScoringPlugin::Buoy::Buoy(
    const gazebo::physics::LinkPtr _buoyLink,
    const BuoyGoal _buoyGoal,
    double _engagementDistance)
  : link(_buoyLink),
    goal(_buoyGoal),
    engagementDistance(_engagementDistance)
{
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
      gzdbg << "[WildlifeScoringPlugin::Buoy] " << this->link->GetName()
            << " Transition from NEVER_ENGAGED" << " to ENGAGED" << std::endl;
    }
  }
  else if (this->state == BuoyState::NOT_ENGAGED)
  {
    if (vehicleBuoyDistance <= this->engagementDistance)
    {
      // Transition to ENGAGED.
      this->state = BuoyState::ENGAGED;
      gzdbg << "[WildlifeScoringPlugin::Buoy] " << this->link->GetName()
            << " Transition from NOT_ENGAGED" << " to ENGAGED" << std::endl;
    }
  }
  else if (this->state == BuoyState::ENGAGED)
  {
    if (vehicleBuoyDistance > this->engagementDistance)
    {
      // Transition to NOT ENGAGED.
      this->state = BuoyState::NOT_ENGAGED;
      gzdbg << "[WildlifeScoringPlugin::Buoy] " << this->link->GetName()
            << " Transition from ENGAGED" << " to NOT_ENGAGED" << std::endl;
    }

    // ToDo: Check if we transition to circumnavigate.
    // ...
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

  // Parse the required <animals_model_name> element.
  if (!_sdf->HasElement("animals_model_name"))
  {
    gzerr << "Unable to find <animals_model_name>" << std::endl;
    return;
  }
  this->animalsModelName = _sdf->Get<std::string>("animals_model_name");

  // Parse the optional <animals_topic> element.
  if (_sdf->HasElement("animals_topic"))
    this->animalsTopic = _sdf->Get<std::string>("animals_topic");

  // Parse the optional <engagement_distance> element.
  if (_sdf->HasElement("engagement_distance"))
    this->engagementDistance = _sdf->Get<double>("engagement_distance");

  // Parse the optional <obstacle_penalty> element.
  if (_sdf->HasElement("obstacle_penalty"))
    this->obstaclePenalty = _sdf->Get<double>("obstacle_penalty");

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
    if (!this->AddBuoy(buoyLinkName, buoyGoal))
      return false;

    // Parse the next buoy.
    buoyElem = buoyElem->GetNextElement("buoy");
  }

  return true;
}

//////////////////////////////////////////////////
bool WildlifeScoringPlugin::AddBuoy(const std::string &_linkName,
    const std::string &_goal)
{
  gazebo::physics::ModelPtr parentModel =
#if GAZEBO_MAJOR_VERSION >= 8
    this->world->ModelByName(this->animalsModelName);
#else
    this->world->GetModel(this->animalsModelName);
#endif
  // Sanity check: Make sure that the model exists.
  if (!parentModel)
  {
    gzerr << "Unable to find model [" << animalsModelName << "]" << std::endl;
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

  // Update the state of all buoys.
  for (auto &buoy : this->buoys)
  {
    // Update the vehicle model if needed.
    if (!buoy.vehicleModel)
      buoy.SetVehicleModel(this->vehicleModel);

    // Update the buoy state.
    buoy.Update();
  }

  // Publish the location of the buoys.
  this->PublishAnimalLocations();
}

//////////////////////////////////////////////////
void WildlifeScoringPlugin::Fail()
{
  this->SetScore(this->ScoringPlugin::GetTimeoutScore());
  this->Finish();
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
    const ignition::math::Vector3d latlon =
      this->world->SphericalCoords()->SphericalFromLocal(pose.Pos());
#else
    const ignition::math::Pose3d pose = buoy.link->GetWorldPose().Ign();
    const ignition::math::Vector3d latlon =
      this->world->GetSphericalCoordinates()->SphericalFromLocal(pose.Pos());
#endif
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
  this->numCollisions++;
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(WildlifeScoringPlugin)
