/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include "vmrc_gazebo/scoring_plugin.hh"

/////////////////////////////////////////////////
ScoringPlugin::ScoringPlugin()
{
}

/////////////////////////////////////////////////
void ScoringPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "ScoringPlugin::Load(): NULL world pointer");
  GZ_ASSERT(_sdf,   "ScoringPlugin::Load(): NULL _sdf pointer");

  this->world = _world;

  // This is a required element.
  if (!_sdf->HasElement("task_name"))
  {
    gzerr << "Unable to find <task_name> element in SDF." << std::endl;
    return;
  }
  this->taskName = _sdf->Get<std::string>("task_name");

  // This is a required element.
  if (!_sdf->HasElement("max_time"))
  {
    gzerr << "Unable to find <max_time> element in SDF." << std::endl;
    return;
  }
  this->maxTime = _sdf->Get<uint32_t>("max_time");

  // Set the end time of the task.
  this->endTime = this->startTime + gazebo::common::Time(this->maxTime, 0);

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ScoringPlugin::Update, this));
}

//////////////////////////////////////////////////
double ScoringPlugin::Score() const
{
  return this->score;
}

//////////////////////////////////////////////////
void ScoringPlugin::SetScore(double newScore)
{
  this->score = newScore;
}

//////////////////////////////////////////////////
std::string ScoringPlugin::TaskName() const
{
  return this->taskName;
}

//////////////////////////////////////////////////
std::string ScoringPlugin::TaskState() const
{
  return this->taskState;
}

//////////////////////////////////////////////////
uint32_t ScoringPlugin::MaxTime() const
{
  return this->maxTime;
}

//////////////////////////////////////////////////
gazebo::common::Time ScoringPlugin::ElapsedTime() const
{
  return this->elapsedTime;
}

//////////////////////////////////////////////////
gazebo::common::Time ScoringPlugin::RemainingTime() const
{
  return this->remainingTime;
}

//////////////////////////////////////////////////
void ScoringPlugin::Update()
{
  // Update time.
  this->currentTime = this->world->GetSimTime();
  this->elapsedTime = std::min(std::max(this->currentTime - this->startTime,
      gazebo::common::Time::Zero), this->endTime - this->startTime);
  this->remainingTime = std::min(std::max(this->endTime - this->currentTime,
      gazebo::common::Time::Zero), this->endTime - this->startTime);

  this->UpdateTaskState();

  // Publish stats.
}

//////////////////////////////////////////////////
void ScoringPlugin::UpdateTaskState()
{
  if (this->taskState == "initial" &&
      this->currentTime >= this->startTime)
  {
    this->taskState = "running";
    this->OnRunning();
    return;
  }

  if (this->taskState == "running" &&
      this->currentTime >= this->endTime)
  {
    this->taskState = "finished";
    this->OnFinished();
    return;
  }  
}

//////////////////////////////////////////////////
void ScoringPlugin::OnRunning()
{
}

//////////////////////////////////////////////////
void ScoringPlugin::OnFinished()
{
}
