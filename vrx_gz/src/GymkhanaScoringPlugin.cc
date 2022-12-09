/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <gz/msgs/param.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <limits>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

#include "GymkhanaScoringPlugin.hh"

using namespace gz;
using namespace vrx;

/// \brief Private GymkhanaScoringPlugin data class.
class GymkhanaScoringPlugin::Implementation
{
  /// \brief Callback for black box station-keeping portion's scoring plugin
  /// \param[in] _msg Task message as defined in ScoringPlugin::Implementation
  public: void BlackboxCallback(const msgs::Param &_msg);

  /// \brief Set the pinger location.
  public: void SetPingerPosition();

  /// \brief Transport node
  public: transport::Node node;

  /// \brief publisher to set the pinger position.
  public: transport::Node::Publisher setPingerPub;

  /// \brief Whether buoy channel portion has been completed
  public: bool channelCrossed = false;

  /// \brief Cumulative error from black box station keeping portion
  public: double blackboxScore = 0.0;

  /// \brief Penalty added per collision.
  public: double obstaclePenalty = 0.1;

  // TODO: compare with acousticpinger
  /// \brief Position of the pinger.
  public: math::Vector3d pingerPosition = {0, 0, 0};

  /// \brief Last time we published a pinger position.
  public: std::chrono::duration<double> lastSetPingerPositionTime = 
    std::chrono::duration<double>::zero();

  /// \brief Pointer to the SDF plugin element.
  public: sdf::ElementPtr sdf;

  /// \brief Display or suppress state changes
  public: bool silent = false;
};

/////////////////////////////////////////////////
void GymkhanaScoringPlugin::ChannelCallback(
  const msgs::Param &_msg)
{
  // Determine whether channel has been crossed before timeout
  if (!this->dataPtr->channelCrossed)
  {
    if (_msg.params().at("state").string_value() == "finished")
    {
      if (_msg.params().at("score").double_value() == 200)
        ScoringPlugin::Finish();
      else
        this->dataPtr->channelCrossed = true;
    }
  }
}

/////////////////////////////////////////////////
void GymkhanaScoringPlugin::Implementation::BlackboxCallback(
  const msgs::Param &_msg)
{
  this->blackboxScore = _msg.params().at("score").double_value();
}

/////////////////////////////////////////////////
void GymkhanaScoringPlugin::Implementation::SetPingerPosition()
{
  msgs::Vector3d position;
  position.set_x(this->pingerPosition.X());
  position.set_y(this->pingerPosition.Y());
  position.set_z(this->pingerPosition.Z());
  this->setPingerPub.Publish(position);
}

/////////////////////////////////////////////////
GymkhanaScoringPlugin::GymkhanaScoringPlugin()
  : ScoringPlugin(),
    dataPtr(utils::MakeUniqueImpl<Implementation>())
{
  gzmsg << "GymkhanaScoringPlugin loaded" << std::endl;
}

/////////////////////////////////////////////////
//TODO
void GymkhanaScoringPlugin::Configure(const sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  sim::EntityComponentManager &_ecm, sim::EventManager &_eventMgr)
{
  // Base class
  ScoringPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  this->dataPtr->sdf = _sdf->Clone();

  // Optional <obstacle_penalty> element.
  if (_sdf->HasElement("obstacle_penalty"))
  {
    this->dataPtr->obstaclePenalty = 
      this->dataPtr->sdf->Get<double>("obstacle_penalty");
  }

  // Throttle messages to 1Hz
  transport::AdvertiseMessageOptions opts;
  opts.SetMsgsPerSec(1u);

  // Set the topic to be used to publish the sensor message.
  std::string setPositionTopicName = "/pinger/set_pinger_position";
  if (_sdf->HasElement("set_position_topic_name"))
  {
    setPositionTopicName =
      this->dataPtr->sdf->GetElement("set_position_topic_name")->Get<std::string>();
  }

  // Set the pinger position.
  if (_sdf->HasElement("pinger_position"))
  {
    this->dataPtr->pingerPosition =
      _sdf->Get<math::Vector3d>("pinger_position");
  }

  this->dataPtr->setPingerPub = 
    this->dataPtr->node.Advertise<msgs::Vector3d>(
    setPositionTopicName, opts);

  this->dataPtr->node.Subscribe("/vrx/gymkhana_channel/task/info",
    &GymkhanaScoringPlugin::ChannelCallback, 
    this);

  this->dataPtr->node.Subscribe("/vrx/gymkhana_blackbox/task/info",
    &GymkhanaScoringPlugin::Implementation::BlackboxCallback, 
    this->dataPtr.get());
}

/////////////////////////////////////////////////
void GymkhanaScoringPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  // don't update when paused
  if (_info.paused)
    return;

  ScoringPlugin::PreUpdate(_info, _ecm);

  std::chrono::duration<double> now = _info.simTime;

  // Set pinger position for the first time if needed
  if (this->dataPtr->lastSetPingerPositionTime == 
      std::chrono::duration<double>::zero())
  {
    this->dataPtr->SetPingerPosition();
    this->dataPtr->lastSetPingerPositionTime = now;
  }

  // Set the pinger position again every 10 seconds.
  std::chrono::duration<double> elapsedTime = 
    now - this->dataPtr->lastSetPingerPositionTime;

  if (elapsedTime >= std::chrono::duration<double>(10.0))
  {
    this->dataPtr->SetPingerPosition();
    this->dataPtr->lastSetPingerPositionTime = now;
  }

  // Nothing to do if the task is not in "running" state.
  if (this->ScoringPlugin::TaskState() != "running")
    return;

  // Check channel navigation is finished
  // If not, invalidate black box station-keeping score
  if (this->dataPtr->channelCrossed)
  {
    this->ScoringPlugin::SetScore(this->dataPtr->blackboxScore);
  }
  else
  {
    this->ScoringPlugin::SetScore(200);
  }
}

//////////////////////////////////////////////////
void GymkhanaScoringPlugin::OnFinished()
{
  // TODO: fix obstacle penalty (Navigation Plugin Implementation)
  double penalty = this->NumCollisions() * this->dataPtr->obstaclePenalty;
  this->SetTimeoutScore(this->Score() + penalty);

  ScoringPlugin::OnFinished();
}

GZ_ADD_PLUGIN(GymkhanaScoringPlugin,
              sim::System,
              ScoringPlugin::ISystemConfigure,
              ScoringPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::GymkhanaScoringPlugin,
                    "vrx::GymkhanaScoringPlugin")
