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

#include <geometry_msgs/Vector3.h>
#include <limits>
#include "vrx_gazebo/gymkhana_scoring_plugin.hh"

/////////////////////////////////////////////////
GymkhanaScoringPlugin::GymkhanaScoringPlugin()
{
  gzmsg << "GymkhanaScoringPlugin loaded" << std::endl;
}

/////////////////////////////////////////////////
GymkhanaScoringPlugin::~GymkhanaScoringPlugin()
{
}

/////////////////////////////////////////////////
void GymkhanaScoringPlugin::Load(gazebo::physics::WorldPtr _world,
  sdf::ElementPtr _sdf)
{
  // Base class, also binds the update method for the base class
  ScoringPlugin::Load(_world, _sdf);

  // Optional <obstacle_penalty> element.
  if (_sdf->HasElement("obstacle_penalty"))
    this->obstaclePenalty = _sdf->Get<double>("obstacle_penalty");

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GymkhanaScoringPlugin::Update, this));

  // Setup ROS node and publisher
  this->rosNode.reset(new ros::NodeHandle());

  // Set the topic to be used to publish the sensor message.
  std::string setPositionTopicName = "/pinger/set_pinger_position";
  if (_sdf->HasElement("set_position_topic_name"))
  {
    setPositionTopicName =
      _sdf->GetElement("set_position_topic_name")->Get<std::string>();
  }

  // Set the pinger position.
  if (_sdf->HasElement("pinger_position"))
  {
    this->pingerPosition =
      _sdf->Get<ignition::math::Vector3d>("pinger_position");
  }

  #if GAZEBO_MAJOR_VERSION >= 8
    this->lastSetPingerPositionTime = this->world->SimTime();
  #else
    this->lastSetPingerPositionTime = this->world->GetSimTime();
  #endif

  this->setPingerPub =
    this->rosNode->advertise<geometry_msgs::Vector3>(
      setPositionTopicName, 1, true);

  this->channelSub = this->rosNode->subscribe(
    "/vrx/gymkhana_channel/task/info", 5,
    &GymkhanaScoringPlugin::ChannelCallback, this);

  this->blackboxSub = this->rosNode->subscribe(
    "/vrx/gymkhana_blackbox/task/info", 5,
    &GymkhanaScoringPlugin::BlackboxCallback, this);
}

/////////////////////////////////////////////////
void GymkhanaScoringPlugin::Update()
{
  // Set the pinger position every 10 seconds.
  #if GAZEBO_MAJOR_VERSION >= 8
    gazebo::common::Time now = this->world->SimTime();
  #else
    gazebo::common::Time now = this->world->GetSimTime();
  #endif
  gazebo::common::Time elapsedTime = now - this->lastSetPingerPositionTime;

  if (elapsedTime.Double() >= 10.0)
  {
    this->SetPingerPosition();
    this->lastSetPingerPositionTime = now;
  }

  // Nothing to do if the task is not in "running" state.
  if (this->ScoringPlugin::TaskState() != "running")
    return;

  // Check channel navigation is finished
  // If not, invalidate black box station-keeping score
  if (this->channelCrossed)
  {
    this->ScoringPlugin::SetScore(this->blackboxScore);
  }
  else
  {
    this->ScoringPlugin::SetScore(200);
  }
}

/////////////////////////////////////////////////
void GymkhanaScoringPlugin::ChannelCallback(
  const vrx_gazebo::Task::ConstPtr& msg)
{
  if (msg)
  {
    // Determine whether channel has been crossed before timeout
    if (!this->channelCrossed)
    {
      if (msg->state == "finished")
      {
        if (msg->score == 200)
          this->Finish();
        else
          this->channelCrossed = true;
      }
    }
  }
}

/////////////////////////////////////////////////
void GymkhanaScoringPlugin::BlackboxCallback(
  const vrx_gazebo::Task::ConstPtr& msg)
{
  if (msg)
  {
    this->blackboxScore = msg->score;
  }
}

/////////////////////////////////////////////////
void GymkhanaScoringPlugin::SetPingerPosition() const
{
  geometry_msgs::Vector3 position;
  position.x = this->pingerPosition.X();
  position.y = this->pingerPosition.Y();
  position.z = this->pingerPosition.Z();
  this->setPingerPub.publish(position);
}

//////////////////////////////////////////////////
void GymkhanaScoringPlugin::OnFinished()
{
  double penalty = this->GetNumCollisions() * this->obstaclePenalty;
  this->SetTimeoutScore(this->Score() + penalty);

  ScoringPlugin::OnFinished();
}

GZ_REGISTER_WORLD_PLUGIN(GymkhanaScoringPlugin)
