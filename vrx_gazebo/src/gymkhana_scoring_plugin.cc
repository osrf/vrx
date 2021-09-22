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

  // Optional <obstacle_penalty> element.
  if (_sdf->HasElement("obstacle_penalty"))
    this->obstaclePenalty = _sdf->Get<double>("obstacle_penalty");

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GymkhanaScoringPlugin::Update, this));

  // Setup publisher
  this->channelSub = this->node->create_subscription<vrx_gazebo::msg::Task>(
    "/vrx/gymkhana_channel/task/info", 5,
    std::bind(&GymkhanaScoringPlugin::ChannelCallback, this, std::placeholders::_1));

  this->blackboxSub = this->node->create_subscription<vrx_gazebo::msg::Task>(
    "/vrx/gymkhana_blackbox/task/info", 5,
    std::bind(&GymkhanaScoringPlugin::BlackboxCallback, this, std::placeholders::_1));
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
  const vrx_gazebo::msg::Task::SharedPtr msg)
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
  const vrx_gazebo::msg::Task::SharedPtr msg)
{
  if (msg)
  {
    this->blackboxScore = msg->score;
  }
}

//////////////////////////////////////////////////
void GymkhanaScoringPlugin::OnFinished()
{
  double penalty = this->GetNumCollisions() * this->obstaclePenalty;

  if (this->Score() < std::numeric_limits<double>::max())
    this->SetTimeoutScore(this->Score() + penalty);

  ScoringPlugin::OnFinished();
}

GZ_REGISTER_WORLD_PLUGIN(GymkhanaScoringPlugin)
