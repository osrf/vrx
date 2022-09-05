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

// #include <geographic_msgs/GeoPoseStamped.h>
// #include <geographic_msgs/GeoPath.h>
// #include <algorithm>
// #include <gazebo/common/Assert.hh>
// #include <gazebo/common/Console.hh>
// #include <gazebo/physics/Link.hh>
// #include <ignition/math/Helpers.hh>
// #include "vrx_gazebo/wildlife_scoring_plugin.hh"

#include "WildlifeScoringPlugin.hh"

using namespace vrx;
  /// \brief All buoy goals.
  private: enum class BuoyGoal
  {
    /// \brief The goal is to stay out of the activation area.
    AVOID,

    /// \brief The goal is to circumnavigate the buoy clockwise.
    CIRCUMNAVIGATE_CLOCKWISE,

    /// \brief The goal is to circumnavigate the buoy counterclockwise
    CIRCUMNAVIGATE_COUNTERCLOCKWISE,
  };

  /// \brief All buoy states.
  private: enum class BuoyState
  {
    /// \brief Not "in" the gate and never engaged.
    NEVER_ENGAGED,

    /// \brief Not "in" the gate but was engaged at some point.
    NOT_ENGAGED,

    /// \brief Inside the area of activation of the gate.
    ENGAGED,

    /// \brief Succesfully circumnavigated.
    CIRCUMNAVIGATED,
  };

  /// \brief All gate states.
  private: enum class GateState
  {
    /// \brief Not "in" the gate.
    VEHICLE_OUTSIDE,

    /// \brief Before the gate.
    VEHICLE_BEFORE,

    /// \brief After the gate.
    VEHICLE_AFTER,

    /// \brief Gate crossed!
    CROSSED,
  };

  /// \brief A virtual gate to help detecting circumnavigation.
  private: class VirtualGate
  {
    /// \brief Constructor.
    /// \param[in] _leftMakerLink The link of the buoy.
    /// \param[in] _offset The offset from the buoy that delimitates the gate.
    /// \param[in] _width The width of the gate.
    public: VirtualGate(const gazebo::physics::LinkPtr _leftMakerLink,
                        const ignition::math::Vector3d &_offset,
                        double _width);

    /// \brief Where is the given robot pose with respect to the gate?
    /// \param _robotWorldPose Pose of the robot, in the world frame.
    /// \return The gate state given the current robot pose.
    public: GateState IsPoseInGate(
      const ignition::math::Pose3d &_robotWorldPose) const;

    /// \brief Recalculate the pose of the gate.
    public: void Update();

    /// \brief The left marker (buoy).
    public: const gazebo::physics::LinkPtr leftMarkerLink;

    /// \brief The offset of the right marker with respect to the left marker
    /// in world coordinates.
    public: const ignition::math::Vector3d offset;

    /// \brief The width of the gate in meters.
    public: const double width;

    /// \brief The center of the gate in the world frame. Note that the roll and
    /// pitch are ignored. Only yaw is relevant and it points into the direction
    /// in which the gate should be crossed.
    public: ignition::math::Pose3d pose;

    /// \brief The state of this gate.
    public: GateState state = GateState::VEHICLE_OUTSIDE;
  };

  /// \brief A buoy that is part of the wildlife task.
  private: class Buoy
  {
    /// \brief Constructor.
    /// \param[in] _buoyLink The buoy's main link.
    /// \param[in] _buoyGoal The buoy's goal.
    /// \param[in] _engagementDistance The vehicle engages with the buoy when
    ///            the distance between them is lower or equal than this value.
    public: Buoy(const gazebo::physics::LinkPtr _buoyLink,
                 const BuoyGoal _buoyGoal,
                 double _engagementDistance);

    /// \brief Update the status of this buoy.
    public: void Update();

    /// \brief Set the vehicle model.
    /// \param[in] _vehicleModel The vehicle model pointer.
    public: void SetVehicleModel(gazebo::physics::ModelPtr _vehicleModel);

    /// \brief The number of virtual gates;
    private: const unsigned int kNumVirtualGates = 8u;

    /// \brief The buoy's main link.
    public: const gazebo::physics::LinkPtr link;

    /// \brief The goal.
    public: const BuoyGoal goal;

    /// \brief The vehicle engages with the buoy when the distance between them
    /// is lower or equal than this value.
    public: const double engagementDistance;

    /// \brief The state of this buoy.
    public: BuoyState state = BuoyState::NEVER_ENGAGED;

    /// \brief Pointer to the vehicle that interacts with the buoy.
    public: ignition::gazebo::Entity vehicleEntity;

    /// \brief A collection of virtual gates around the buoy.
    public: std::vector<VirtualGate> virtualGates;

    /// \brief The number of virtual gates currently crossed.
    public: unsigned int numVirtualGatesCrossed = 0u;
  };

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
  if (this->state == BuoyState::CIRCUMNAVIGATED || !this->vehicleEntity)
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

/// \brief Private WildlifeScoringPlugin data class.
class WildlifeScoringPlugin::Implementation
{
    /// \brief Parse the buoys from SDF.
  /// \param[in] _sdf The current SDF element.
  /// \return True when the buoys were successfully parsed or false otherwise.
  private: bool ParseBuoys(sdf::ElementPtr _sdf);

  /// \brief Register a new buoy.
  /// \param[in] _modelName The name of the buoy's model.
  /// \param[in] _linkName The name of the main buoy's link.
  /// \param[in] _goal The goal associated to this buoy.
  /// \return True when the buoy has been registered or false otherwise.
  private: bool AddBuoy(const std::string &_modelName,
                        const std::string &_linkName,
                        const std::string &_goal);
    /// \brief Publish a new message with the animal locations.
  private: void PublishAnimalLocations();

  /// \brief Compute the total bonus achieved.
  /// \return The time bonus in seconds.
  private: double TimeBonus() const;
    /// \brief Transport node.
public:
    ignition::transport::Node node;

    /// \brief Publisher for the animal buoys.
public:
    ignition::transport::Node::Publisher animalsPub;

    /// \brief Spherical coordinates conversions.
public:
    ignition::math::SphericalCoordinates sc;

    /// \brief Vehicle to score.
public:
    ignition::gazebo::Entity vehicleEntity;

     /// \brief All the buoys.
  private: std::vector<Buoy> buoys;

  /// \brief The name of the topic where the animal locations are published.
  private: std::string animalsTopic = "/vrx/wildlife_animals";

  /// \brief Time bonus granted for each succcesfuly goal achieved.
  private: double timeBonus = 30.0;

  /// \brief When the vehicle is between the buoy and this distance, the vehicle
  /// engages with the buoy.
  private: double engagementDistance = 10.0;

  /// \brief True when a vehicle collision is detected.
  private: std::atomic<bool> collisionDetected{false};
};


//////////////////////////////////////////////////
bool WildlifeScoringPlugin::ParseBuoys(sdf::ElementPtr _sdf)
{
  // We need at least one buoy.
  if (!_sdf->HasElement("buoy"))
  {
    ignerr << "Unable to find <buoy> element in SDF." << std::endl;
    return false;
  }

  auto buoyElem = _sdf->GetElement("buoy");

  // Parse a new buoy.
  while (buoyElem)
  {
    // The buoy's model.
    if (!buoyElem->HasElement("model_name"))
    {
      ignerr << "Unable to find <buoys><buoy><model_name> element in SDF."
            << std::endl;
      return false;
    }

    const std::string buoyModelName = buoyElem->Get<std::string>("model_name");

    // The buoy's main link.
    if (!buoyElem->HasElement("link_name"))
    {
      ignerr << "Unable to find <buoys><buoy><link_name> element in SDF."
            << std::endl;
      return false;
    }

    const std::string buoyLinkName = buoyElem->Get<std::string>("link_name");

    // The buoy's goal.
    if (!buoyElem->HasElement("goal"))
    {
      ignerr << "Unable to find <buoys><buoy><goal> element in SDF."
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
    // // The vehicle might not be ready yet, let's try to get it.
    // if (!this->dataPtr->vehicleEntity)
    // {
    //     auto entity = _ecm.EntityByComponents(
    //         ignition::gazebo::components::Name(ScoringPlugin::VehicleName()));
    //     if (entity != ignition::gazebo::kNullEntity)
    //         this->dataPtr->vehicleEntity = entity;
    //     else
    //         return;
    // }

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
    ignerr << "Unable to find link [" << _linkName << "]" << std::endl;
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
    ignerr << "Unknown <goal> value: [" << _goal << "]" << std::endl;
    return false;
  }

  // Save the new buoy.
  dataPtr->buoys.push_back(Buoy(link, buoyGoal, dataPtr->engagementDistance));

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
  dataPtr->SetTimeoutScore(std::max(0.0, dataPointer->Score() - dataPtr->TimeBonus()));
  ScoringPlugin::OnFinished();
}
/////////////////////////////////////////////////
WildlifeScoringPlugin::WildlifeScoringPlugin()
    : ScoringPlugin(), 
    dataPtr(ignition::utils::MakeUniqueImpl<Implementation>())
{
    ignmsg << "Wildlife scoring plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void StationkeepingScoringPlugin::Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr)
{
    ScoringPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
    ignmsg << "Task [" << this->TaskName() << "]" << std::endl;
    auto worldEntity =
        _ecm.EntityByComponents(ignition::gazebo::components::World());
    ignition::gazebo::World world(worldEntity);
    this->dataPtr->sc = world.SphericalCoordinates(_ecm).value();

  // Parse the optional <animals_topic> element.
    
    if (_sdf->HasElement("animals_topic"))
    {
        dataPtr->animalsTopic = _sdf->Get<std::string>("animals_topic");
    }
    // Publish animal buoy pose vector
    dataPtr->goalPub =
        dataPtr->node.Advertise<ignition::msgs::Pose_V>(
            dataPtr->animalsTopic);
  // Parse the optional <engagement_distance> element.
  if (_sdf->HasElement("engagement_distance"))
    dataPtr->engagementDistance = _sdf->Get<double>("engagement_distance");

  // Parse the optional <time_bonus> element.
  if (_sdf->HasElement("time_bonus"))
    dataPtr->timeBonus = _sdf->Get<double>("time_bonus");

  // Parse the optional <buoys> element.
  // Note: Parse this element at the end because we use some of the previous
  // parameters.
  if (_sdf->HasElement("buoys"))
  {
    auto const &buoysElem = _sdf->GetElement("buoys");
    if (!this->ParseBuoys(buoysElem))
    {
      ignerr << "Score has been disabled" << std::endl;
      return;
    }
  }

  ignmsg << "Task [" << this->TaskName() << "]" << std::endl;
}


// Register plugin with 
IGNITION_ADD_PLUGIN(vrx::WildlifeScoringPlugin,
                    ignition::gazebo::System,
                    vrx::StationkeepingScoringPlugin::ISystemConfigure,
                    vrx::StationkeepingScoringPlugin::ISystemPreUpdate)
