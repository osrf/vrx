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
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
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
  this->sdf = _sdf;

  // This is a required element.
  if (!_sdf->HasElement("vehicle"))
  {
    gzerr << "Unable to find <vehicle> element in SDF." << std::endl;
    return;
  }
  this->vehicleName = _sdf->Get<std::string>("vehicle");

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

  // This is an optional element.
  if (_sdf->HasElement("topic"))
    this->topic = _sdf->Get<std::string>("topic");

  // These are optional elements.
  if (_sdf->HasElement("release_joints"))
  {
    auto releaseJointsElem = _sdf->GetElement("release_joints");

    // We need at least one joint.
    if (!releaseJointsElem->HasElement("joint"))
    {
      gzerr << "Unable to find <joint> element in SDF." << std::endl;
      return;
    }

    auto jointElem = releaseJointsElem->GetElement("joint");

    // Parse a new joint to be released.
    while (jointElem)
    {
      // The joint's name.
      if (!jointElem->HasElement("name"))
      {
        gzerr << "Unable to find <name> element in SDF." << std::endl;
        return;
      }

      const std::string jointName = jointElem->Get<std::string>("name");
      this->lockJointNames.push_back(jointName);

      // Parse the next gate.
      jointElem = jointElem->GetNextElement("joint");
    }
  }

  // Set the end time of the task.
  this->endTime = this->runningTime + gazebo::common::Time(this->maxTime, 0);

  // Prepopulate the task msg.
  this->taskMsg.name = this->taskName;
  this->taskMsg.start_time.fromSec(this->runningTime.Double());
  this->UpdateTaskMessage();

  // Initialize ROS transport.
  this->rosNode.reset(new ros::NodeHandle());
  this->taskPub = this->rosNode->advertise<vmrc_gazebo::Task>(this->topic, 100);

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
void ScoringPlugin::Finish()
{
  if (this->taskState == "finished")
    return;

  this->taskState = "finished";
  this->OnFinished();
}

//////////////////////////////////////////////////
void ScoringPlugin::Update()
{
  // The vehicle might not be ready yet, let's try to get it.
  if (!this->vehicleModel)
    this->vehicleModel = this->world->GetModel(this->vehicleName);

  // Update time.
  this->currentTime = this->world->GetSimTime();

  if (this->taskState == "running")
  {
    this->elapsedTime = std::min(std::max(this->currentTime - this->runningTime,
        gazebo::common::Time::Zero), this->endTime - this->runningTime);
    this->remainingTime = std::min(std::max(this->endTime - this->currentTime,
        gazebo::common::Time::Zero), this->endTime - this->runningTime);
    this->timedOut = this->remainingTime <= gazebo::common::Time::Zero;
  }

  this->UpdateTaskState();

  // Publish stats.
  this->PublishStats();
}

//////////////////////////////////////////////////
void ScoringPlugin::UpdateTaskState()
{
  if (this->taskState == "initial" &&
      this->currentTime >= this->readyTime)
  {
    this->taskState = "ready";
    this->ReleaseVehicle();
    this->OnReady();

    return;
  }

  if (this->taskState == "ready" &&
      this->currentTime >= this->runningTime)
  {
    this->taskState = "running";
    this->OnRunning();
    return;
  }

  if (this->taskState == "running" && this->timedOut)
  {
    this->taskState = "finished";
    this->OnFinished();
    return;
  }  
}

//////////////////////////////////////////////////
void ScoringPlugin::UpdateTaskMessage()
{
  this->taskMsg.state = this->taskState;
  this->taskMsg.elapsed_time.fromSec(this->elapsedTime.Double());
  this->taskMsg.remaining_time.fromSec(this->remainingTime.Double());
  this->taskMsg.timed_out = this->timedOut;
  this->taskMsg.finished = this->taskState == "finished";
}

//////////////////////////////////////////////////
void ScoringPlugin::PublishStats()
{
  this->UpdateTaskMessage();

  // We publish stats at 1Hz.
  if (this->currentTime - this->lastStatsSent >= gazebo::common::Time(1, 0))
  {
    this->taskPub.publish(this->taskMsg);
    this->lastStatsSent = this->currentTime;
  }
}

//////////////////////////////////////////////////
void ScoringPlugin::ReleaseVehicle()
{
  if (!this->vehicleModel || this->lockJointsNames.empty())
    return;

  for (auto jointName : this->lockJointNames)
  {
    auto joint = this->vehicleModel->GetJoint(jointName);
    if (joint)
      joint->Detach();
    else
      gzerr << "Unable to release [" << jointName << "]" << std::endl;
  }

  this->lockJointNames.clear();

  gzmsg << "Vehicle released" << std::endl;
}

//////////////////////////////////////////////////
void ScoringPlugin::OnReady()
{
}

//////////////////////////////////////////////////
void ScoringPlugin::OnRunning()
{
}

//////////////////////////////////////////////////
void ScoringPlugin::OnFinished()
{
}
