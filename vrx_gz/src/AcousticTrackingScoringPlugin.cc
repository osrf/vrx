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

#include <gz/msgs/float.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/World.hh>

#include "AcousticTrackingScoringPlugin.hh"
#include "WaypointMarkers.hh"

using namespace gz;
using namespace vrx;

/// \brief Private AcousticMovingScoringPlugin data class.
class AcousticTrackingScoringPlugin::Implementation
{
  /// \brief A transport node.
  public: transport::Node node;

  /// \brief Pointer to the SDF plugin element.
  public: sdf::ElementPtr sdf;

  /// \brief Waypoint visualization markers
  public: WaypointMarkers waypointMarkers{"pinger_marker"};

  /// \brief Entity of the vehicle used.
  public: sim::Entity vehicleEntity;

  /// \brief Entity of the pinger.
  public: sim::Entity pingerEntity;

  /// \brief Pinger model name.
  public: std::string pingerModelName = "pinger";

  /// \brief Pinger depth.
  public: double pingerDepth = 2.0;

  /// \brief Publisher for the pinger location.
  public: transport::Node::Publisher waypointPub;

  /// \brief Publisher for the combined 2D pose error.
  public: transport::Node::Publisher poseErrorPub;

  /// \brief Publisher for the current mean error.
  public: transport::Node::Publisher meanErrorPub;

  /// \brief Combined 2D pose error (distance and yaw).
  public: double poseError;

  /// \brief Number of instant pose error scores calculated so far.
  public: unsigned int sampleCount = 0;

  /// \brief Sum of all pose error scores calculated so far.
  public: double totalPoseError = 0;

  /// \brief Cumulative 2D RMS error in meters.
  public: double meanError;

  /// \brief Penalty added per collision.
  public: double obstaclePenalty = 1.0;

  /// /brief The topic used to set the pinger position.
  public: std::string setPingerTopicName = "/pinger/set_pinger_position";

  /// \brief Topic where 2D pose error is published.
  public: std::string poseErrorTopic = "/vrx/acoustic_wayfinding/pose_error";

  /// \brief Topic where mean pose error is published.
  public: std::string meanErrorTopic =
    "/vrx/acoustic_wayfinding/mean_pose_error";
};

/////////////////////////////////////////////////
AcousticTrackingScoringPlugin::AcousticTrackingScoringPlugin()
  : ScoringPlugin(),
  dataPtr(utils::MakeUniqueImpl<Implementation>())
{
  gzmsg << "AcousticTrackingScoringPlugin scoring plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void AcousticTrackingScoringPlugin::Configure(const sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  sim::EntityComponentManager &_ecm, sim::EventManager &_eventMgr)
{
  ScoringPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  this->dataPtr->sdf = _sdf->Clone();

  // Optional set pinger position topic name.
  if (_sdf->HasElement("set_pinger_position_topic"))
  {
    this->dataPtr->setPingerTopicName =
      _sdf->Get<std::string>("set_pinger_position_topic");
  }

  // Optional Pinger model name.
  if (_sdf->HasElement("pinger_model"))
    this->dataPtr->pingerModelName = _sdf->Get<std::string>("pinger_model");

  // Optional Pinger depth.
  if (_sdf->HasElement("pinger_depth"))
    this->dataPtr->pingerDepth = _sdf->Get<double>("pinger_depth");

  // Optional pose error topic.
  if (_sdf->HasElement("pose_error_topic"))
    this->dataPtr->poseErrorTopic = _sdf->Get<std::string>("pose_error_topic");

  // Optional mean error topic.
  if (_sdf->HasElement("mean_error_topic"))
    this->dataPtr->meanErrorTopic = _sdf->Get<std::string>("mean_error_topic");

  // Optional <obstacle_penalty> element.
  if (_sdf->HasElement("obstacle_penalty"))
  {
    this->dataPtr->obstaclePenalty = 
      this->dataPtr->sdf->Get<double>("obstacle_penalty");
  }

  // Throttle messages to 1Hz.
  transport::AdvertiseMessageOptions opts1;
  opts1.SetMsgsPerSec(1u);

  // Throttle messages to 10Hz.
  transport::AdvertiseMessageOptions opts10;
  opts10.SetMsgsPerSec(10u);

  // The publisher to update the pinger position.
  this->dataPtr->waypointPub = this->dataPtr->node.Advertise<msgs::Vector3d>(
    this->dataPtr->setPingerTopicName, opts10);

  // The publisher to update the pose error.
  this->dataPtr->poseErrorPub = this->dataPtr->node.Advertise<msgs::Float>(
    this->dataPtr->poseErrorTopic, opts1);

  // The publisher to update the mean error.
  this->dataPtr->meanErrorPub = this->dataPtr->node.Advertise<msgs::Float>(
    this->dataPtr->meanErrorTopic, opts1);
}

//////////////////////////////////////////////////
void AcousticTrackingScoringPlugin::PreUpdate( const sim::UpdateInfo &_info,
  sim::EntityComponentManager &_ecm)
{
  // Don't update when paused
  if (_info.paused)
    return;

  ScoringPlugin::PreUpdate(_info, _ecm);

  if (this->ScoringPlugin::TaskState() != "running")
    return;

  // The vehicle might not be ready yet, let's try to get it.
  if (!this->dataPtr->vehicleEntity)
  {
    auto entity = _ecm.EntityByComponents(
      sim::components::Name(ScoringPlugin::VehicleName()));
    if (entity == sim::kNullEntity)
      return;

    this->dataPtr->vehicleEntity = entity;
  }

  // The pinger might not be ready yet, let's try to get it.
  if (!this->dataPtr->pingerEntity)
  {
    auto entity = _ecm.EntityByComponents(
      sim::components::Name(this->dataPtr->pingerModelName));
    if (entity == sim::kNullEntity)
      return;

    this->dataPtr->pingerEntity = entity;
  }

  // Update the pinger position.
  auto pingerPose = _ecm.Component<sim::components::Pose>(
    this->dataPtr->pingerEntity)->Data();
  pingerPose.Pos().Z(this->dataPtr->pingerDepth);
  msgs::Vector3d msg;
  msg.set_x(pingerPose.Pos().X());
  msg.set_y(pingerPose.Pos().Y());
  msg.set_z(pingerPose.Pos().Z());
  this->dataPtr->waypointPub.Publish(msg);

  // Read vehicle pose.
  auto vehiclePose = _ecm.Component<sim::components::Pose>(
    this->dataPtr->vehicleEntity)->Data();

  // Update stats.
  double dx = pingerPose.Pos().X() - vehiclePose.Pos().X();
  double dy = pingerPose.Pos().Y() - vehiclePose.Pos().Y();
  double dist = sqrt(pow(dx, 2) + pow(dy, 2));

  this->dataPtr->poseError = dist;
  this->dataPtr->totalPoseError += this->dataPtr->poseError;
  this->dataPtr->sampleCount++;
  this->dataPtr->meanError =
    this->dataPtr->totalPoseError / this->dataPtr->sampleCount;

  // Publish stats.
  msgs::Float poseErrorMsg;
  msgs::Float meanErrorMsg;
  poseErrorMsg.set_data(this->dataPtr->poseError);
  meanErrorMsg.set_data(this->dataPtr->meanError);

  this->dataPtr->poseErrorPub.Publish(poseErrorMsg);
  this->dataPtr->meanErrorPub.Publish(meanErrorMsg);

  // Update the score.
  ScoringPlugin::SetScore(this->dataPtr->meanError);
}

//////////////////////////////////////////////////
void AcousticTrackingScoringPlugin::OnFinished()
{
  double penalty = this->NumCollisions() * this->dataPtr->obstaclePenalty;
  this->SetTimeoutScore(this->Score() + penalty);

  ScoringPlugin::OnFinished();
}

GZ_ADD_PLUGIN(vrx::AcousticTrackingScoringPlugin,
              sim::System,
              vrx::AcousticTrackingScoringPlugin::ISystemConfigure,
              vrx::AcousticTrackingScoringPlugin::ISystemPreUpdate)
