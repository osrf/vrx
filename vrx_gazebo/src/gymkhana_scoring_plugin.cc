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

#include "vrx_gazebo/gymkhana_scoring_plugin.hh"
#include <limits>

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

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GymkhanaScoringPlugin::Update, this));

  // Setup ROS node and publisher
  this->rosNode.reset(new ros::NodeHandle());

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
    this->ScoringPlugin::SetScore(std::numeric_limits<double>::max());
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
      if (msg->state == "finished" && !msg->timed_out)
      {
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

GZ_REGISTER_WORLD_PLUGIN(GymkhanaScoringPlugin)
