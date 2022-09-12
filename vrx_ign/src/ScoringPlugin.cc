/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <ignition/msgs/param.pb.h>
#include <chrono>
#include <string>
#include <vector>
#include <ignition/common/Profiler.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/Events.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/sdf.hh>

#include "ScoringPlugin.hh"

using namespace ignition;
using namespace vrx;

/// \brief Private ScoringPlugin data class.
class ScoringPlugin::Implementation
{
  /// \brief Parse all SDF parameters.
  /// \return True when all parameters were successfully parsed or false
  /// otherwise.
  public: bool ParseSDFParameters();

  /// \brief Update the task stats message.
  public: void UpdateTaskMessage();

  /// \brief Update all time-related variables.
  /// \param[in] _simTime Current simulation time.
  public: void UpdateTime(const std::chrono::duration<double> _simTime);

  /// \brief Publish the task stats over a topic.
  public: void PublishStats();

  /// \brief Shutdown Gazebo and ROS.
  public: void Exit();

  /// \brief The name of the task.
  public: std::string taskName = "undefined";

  /// \brief The name of the vehicle to score.
  public: std::string vehicleName;

  /// \brief Vehicle to score.
  public: gazebo::Model vehicleModel;

  /// \brief Silent mode enabled?
  public: bool silent = false;

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief Transport node publisher for task information.
  public: transport::Node::Publisher taskPub;

  /// \brief Transport node publisher for releasing the vehicle.
  public: transport::Node::Publisher releasePub;

  /// \brief Topic where the task stats are published.
  public: std::string taskInfoTopic = "/vrx/task/info";

  /// \brief Topic name to release the vehicle if locked to the world.
  public: std::string releaseTopic = "/vrx/release";

  /// \brief Bool flag for debug.
  public: bool debug = true;

  /// \brief The score.
  public: double score = 0.0;

  /// \brief Pointer to the SDF plugin element.
  public: sdf::ElementPtr sdf;

  /// \brief Duration (seconds) of the initial state.
  public: double initialStateDuration = 30.0;

  /// \brief Duration (seconds) of the ready state.
  public: double readyStateDuration = 60.0;

  /// \brief Duration (seconds) of the running state (max task time).
  public: double runningStateDuration = 300.0;

  /// \brief Absolute time specifying the start of the ready state.
  public: std::chrono::duration<double> readyTime;

  /// \brief Absolute time specifying the start of the running state.
  public: std::chrono::duration<double> runningTime;

  /// \brief Absolute time specifying the start of the finish state.
  public: std::chrono::duration<double> finishTime;

  /// \brief Current time (simulation).
  public: std::chrono::duration<double> currentTime;

  // \brief Elapsed time since the start of the task (running state).
  public: std::chrono::duration<double> elapsedTime;

  /// \brief Remaining time since the start of the task (running state).
  public: std::chrono::duration<double> remainingTime;

  /// \brief Last time we considered a vehicle collision.
  public: std::chrono::duration<double> lastCollisionTime;

  /// \brief Whether the current task has timed out or not.
  public: bool timedOut = false;

  /// \brief Time at which the last message was sent.
  public: std::chrono::duration<double> lastStatsSent{0};

  /// \brief The task state.
  public: std::string taskState = "initial";

  /// \brief The next task message to be published.
  public: msgs::Param taskMsg;

  /// \brief Score in case of timeout - added for Navigation task
  public: double timeoutScore = -1;

  /// \brief Whether to shut down after last gate is crossed.
  public: bool perPluginExitOnCompletion = true;

  /// \brief Collision buffer.
  public: double collisionBuffer = 3.0;

  /// \brief Collision list.
  public: std::vector<std::string> collisionList;

  /// \brief Collisions timestamps.
  public: std::vector<double> collisionTimestamps;

  /// \brief Number of vehicle collisions.
  public: uint16_t numCollisions = 0u;

  /// \brief Event manager for exiting the simulation.
  public: gazebo::EventManager *eventManager{nullptr};
};

//////////////////////////////////////////////////
bool ScoringPlugin::Implementation::ParseSDFParameters()
{
  // This is a required element.
  if (!this->sdf->HasElement("vehicle"))
  {
    ignerr << "Unable to find <vehicle> element in SDF." << std::endl;
    return false;
  }
  this->vehicleName = this->sdf->Get<std::string>("vehicle");

  // This is a required element.
  if (!this->sdf->HasElement("task_name"))
  {
    ignerr << "Unable to find <task_name> element in SDF." << std::endl;
    return false;
  }
  this->taskName = this->sdf->Get<std::string>("task_name");

  // This is an optional element.
  if (this->sdf->HasElement("task_info_topic"))
    this->taskInfoTopic = this->sdf->Get<std::string>("task_info_topic");

  // This is an optional element.
  if (this->sdf->HasElement("per_plugin_exit_on_completion"))
  {
    this->perPluginExitOnCompletion = this->sdf->Get<bool>(
      "per_plugin_exit_on_completion");
  }

  // This is an optional element.
  if (this->sdf->HasElement("initial_state_duration"))
  {
    double value = this->sdf->Get<double>("initial_state_duration");
    if (value < 0)
    {
      ignerr << "<initial_state_duration> value should not be negative."
             << std::endl;
      return false;
    }
    this->initialStateDuration = value;
  }

  // This is an optional element.
  if (this->sdf->HasElement("ready_state_duration"))
  {
    double value = this->sdf->Get<double>("ready_state_duration");
    if (value < 0)
    {
      ignerr << "<ready_state_duration> value should not be negative."
             << std::endl;
      return false;
    }

    this->readyStateDuration = value;
  }

  // This is an optional element.
  if (this->sdf->HasElement("running_state_duration"))
  {
    double value = this->sdf->Get<double>("running_state_duration");
    if (value < 0)
    {
      ignerr << "<running_state_duration> value should not be negative."
             << std::endl;
      return false;
    }
    this->runningStateDuration = value;
  }

  // This is an optional element.
  if (this->sdf->HasElement("release_topic"))
  {
    this->releaseTopic = this->sdf->Get<std::string>("release_topic");
  }

  // This is an optional element.
  if (this->sdf->HasElement("silent"))
  {
    this->silent = this->sdf->Get<bool>("silent");
  }

  return true;
}

//////////////////////////////////////////////////
void ScoringPlugin::Implementation::UpdateTime(
  const std::chrono::duration<double> _simTime)
{
  this->currentTime = _simTime;

  if (this->taskState == "running")
  {
    this->elapsedTime = this->currentTime - this->runningTime;
    this->remainingTime = this->finishTime - this->currentTime;
    this->timedOut = this->remainingTime <= std::chrono::duration<double>(0.0);
  }
}

//////////////////////////////////////////////////
void ScoringPlugin::UpdateTaskState()
{
  if (this->dataPtr->taskState == "initial" &&
      this->dataPtr->currentTime >= this->dataPtr->readyTime)
  {
    this->dataPtr->taskState = "ready";
    this->ReleaseVehicle();
    this->OnReady();
    return;
  }

  if (this->dataPtr->taskState == "ready" &&
      this->dataPtr->currentTime >= this->dataPtr->runningTime)
  {
    this->dataPtr->taskState = "running";
    this->OnRunning();
    return;
  }

  if (this->dataPtr->taskState == "running" && this->dataPtr->timedOut)
  {
    this->dataPtr->taskState = "finished";
    this->OnFinished();
    return;
  }
}

//////////////////////////////////////////////////
void ScoringPlugin::Implementation::UpdateTaskMessage()
{
  auto *param = this->taskMsg.mutable_params();

  (*param)["state"].set_string_value(this->taskState);
  (*param)["elapsed_time"].set_double_value(this->elapsedTime.count());
  (*param)["remaining_time"].set_double_value(this->remainingTime.count());
  (*param)["timed_out"].set_bool_value(this->timedOut);
  (*param)["num_collisions"].set_int_value(this->numCollisions);
  (*param)["score"].set_double_value(this->score);
}

//////////////////////////////////////////////////
void ScoringPlugin::Implementation::PublishStats()
{
  this->UpdateTaskMessage();

  // We publish stats at 1Hz.
  if (this->currentTime - this->lastStatsSent >=
      std::chrono::duration<double>(1.0))
  {
    this->taskPub.Publish(this->taskMsg);
    this->lastStatsSent = this->currentTime;
  }
}

//////////////////////////////////////////////////
void ScoringPlugin::Implementation::Exit()
{
  bool exit = this->perPluginExitOnCompletion;

  char* env = std::getenv("VRX_EXIT_ON_COMPLETION");
  if (env != nullptr && std::string(env) == "true")
  {
    // Overwrite class variable if environment variable is specified
    exit = true;
  }

  if (exit)
  {
    // shutdown gazebo
    this->eventManager->Emit<gazebo::events::Stop>();
  }
  else
  {
    ignerr << "VRX_EXIT_ON_COMPLETION and <per_plugin_exit_on_completion> "
           << "both not set, will not shutdown on ScoringPlugin::Exit()"
           << std::endl;
  }
  return;
}

//////////////////////////////////////////////////
ScoringPlugin::ScoringPlugin()
  : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void ScoringPlugin::Configure(const gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gazebo::EntityComponentManager &_ecm,
    gazebo::EventManager &_eventMgr)
{
  this->dataPtr->sdf = _sdf->Clone();
  this->dataPtr->eventManager = &_eventMgr;

  // SDF.
  if (!this->dataPtr->ParseSDFParameters())
  {
    ignerr << "Scoring disabled" << std::endl;
    return;
  }

  this->dataPtr->readyTime =
    std::chrono::duration<double>(this->dataPtr->initialStateDuration);
  this->dataPtr->runningTime = this->dataPtr->readyTime +
    std::chrono::duration<double>(this->dataPtr->readyStateDuration);
  this->dataPtr->finishTime =this->dataPtr->runningTime +
    std::chrono::duration<double>(this->dataPtr->runningStateDuration);

  // Prepopulate the task msg.
  auto *param = this->dataPtr->taskMsg.mutable_params();

  msgs::Any nameValue;
  nameValue.set_type(msgs::Any_ValueType::Any_ValueType_STRING);
  nameValue.set_string_value(this->dataPtr->taskName);
  (*param)["name"] = nameValue;

  msgs::Any stateValue;
  stateValue.set_type(msgs::Any_ValueType::Any_ValueType_STRING);
  (*param)["state"] = stateValue;

  msgs::Any readyTimeValue;
  readyTimeValue.set_type(msgs::Any_ValueType::Any_ValueType_DOUBLE);
  readyTimeValue.set_double_value(this->dataPtr->readyTime.count());
  (*param)["ready_time"] = readyTimeValue;

  msgs::Any runningTimeValue;
  runningTimeValue.set_type(msgs::Any_ValueType::Any_ValueType_DOUBLE);
  runningTimeValue.set_double_value(this->dataPtr->runningTime.count());
  (*param)["running_time"] = runningTimeValue;

  msgs::Any elapsedTimeValue;
  elapsedTimeValue.set_type(msgs::Any_ValueType::Any_ValueType_DOUBLE);
  (*param)["elapsed_time"] = elapsedTimeValue;

  msgs::Any remainingTimeValue;
  remainingTimeValue.set_type(msgs::Any_ValueType::Any_ValueType_DOUBLE);
  (*param)["remaining_time"] = remainingTimeValue;

  msgs::Any timedOutValue;
  timedOutValue.set_type(msgs::Any_ValueType::Any_ValueType_DOUBLE);
  (*param)["timed_out"] = timedOutValue;

  msgs::Any numCollisionsValue;
  numCollisionsValue.set_type(msgs::Any_ValueType::Any_ValueType_INT32);
  (*param)["num_collisions"] = numCollisionsValue;

  msgs::Any scoreValue;
  scoreValue.set_type(msgs::Any_ValueType::Any_ValueType_DOUBLE);
  (*param)["score"] = scoreValue;

  this->dataPtr->UpdateTaskMessage();

  this->dataPtr->taskPub = this->dataPtr->node.Advertise<msgs::Param>(
    this->dataPtr->taskInfoTopic);

  this->dataPtr->releasePub = this->dataPtr->node.Advertise<msgs::Empty>(
    this->dataPtr->releaseTopic);

  if (char *envDbg = std::getenv("VRX_DEBUG"))
  {
    if (std::string(envDbg) == "false")
      this->dataPtr->debug = false;
  }
  else
  {
    ignwarn << "VRX_DEBUG environment variable not set, defaulting to true"
            << std::endl;
  }
}

//////////////////////////////////////////////////
void ScoringPlugin::PreUpdate(const gazebo::UpdateInfo &_info,
    gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("ScoringPlugin::PreUpdate");

  this->dataPtr->UpdateTime(_info.simTime);
  this->UpdateTaskState();
  this->dataPtr->PublishStats();
}

//////////////////////////////////////////////////
double ScoringPlugin::Score() const
{
  return this->dataPtr->score;
}

//////////////////////////////////////////////////
void ScoringPlugin::SetScore(double _newScore)
{
  if (this->TaskState() == "running")
    this->dataPtr->score = _newScore;
}

//////////////////////////////////////////////////
std::string ScoringPlugin::TaskName() const
{
  return this->dataPtr->taskName;
}

//////////////////////////////////////////////////
std::string ScoringPlugin::TaskState() const
{
  return this->dataPtr->taskState;
}

//////////////////////////////////////////////////
double ScoringPlugin::RunningStateDuration() const
{
  return this->dataPtr->runningStateDuration;
}

//////////////////////////////////////////////////
std::chrono::duration<double> ScoringPlugin::ElapsedTime() const
{
  return this->dataPtr->elapsedTime;
}

//////////////////////////////////////////////////
std::chrono::duration<double> ScoringPlugin::RemainingTime() const
{
  return this->dataPtr->remainingTime;
}

//////////////////////////////////////////////////
void ScoringPlugin::SetTimeoutScore(double _timeoutScore)
{
  this->dataPtr->timeoutScore = _timeoutScore;
}

//////////////////////////////////////////////////
double ScoringPlugin::TimeoutScore() const
{
  return this->dataPtr->timeoutScore;
}

//////////////////////////////////////////////////
std::string ScoringPlugin::VehicleName() const
{
  return this->dataPtr->vehicleName;
}

//////////////////////////////////////////////////
uint16_t ScoringPlugin::NumCollisions() const
{
  return this->dataPtr->numCollisions;
}

//////////////////////////////////////////////////
void ScoringPlugin::OnReady()
{
  if (!this->dataPtr->silent)
    igndbg << "ScoringPlugin::OnReady" << std::endl;
}

//////////////////////////////////////////////////
void ScoringPlugin::OnRunning()
{
  if (!this->dataPtr->silent)
    igndbg << "ScoringPlugin::OnRunning" << std::endl;
}

//////////////////////////////////////////////////
void ScoringPlugin::OnFinished()
{
  if (!this->dataPtr->silent)
    igndbg << this->dataPtr->currentTime.count() << "  OnFinished" << std::endl;

  // If a timeoutScore was specified, use it.
  if (this->dataPtr->timedOut && this->dataPtr->timeoutScore > 0.0)
  {
    this->dataPtr->score = this->dataPtr->timeoutScore;
  }
  this->dataPtr->UpdateTaskMessage();
  this->dataPtr->taskPub.Publish(this->dataPtr->taskMsg);

  this->dataPtr->Exit();
}

//////////////////////////////////////////////////
void ScoringPlugin::OnCollision()
{
}

//////////////////////////////////////////////////
void ScoringPlugin::OnContacts(const ignition::msgs::Contacts &_contacts)
{
  // If any collision includes the WAM-V, increment collision counter.
  for (unsigned int i = 0; i < _contacts.contact_size(); ++i)
  {
    std::string wamvCollisionStr1 = _contacts.contact(i).collision1().name();
    std::string wamvCollisionStr2 = _contacts.contact(i).collision2().name();

    bool isWamvHit =
      wamvCollisionStr1.find("wamv::base_link::") != std::string::npos ||
      wamvCollisionStr2.find("wamv::base_link::") != std::string::npos;

    bool isHitBufferPassed =
      (this->dataPtr->currentTime - this->dataPtr->lastCollisionTime).count() >
      this->dataPtr->collisionBuffer;

    if (isWamvHit && isHitBufferPassed)
    {
      ++this->dataPtr->numCollisions;
      if (!this->dataPtr->silent)
      {
        igndbg << "[" << this->dataPtr->numCollisions
               << "] New collision counted between ["
               << _contacts.contact(i).collision1().name() << "] and ["
               << _contacts.contact(i).collision2().name() << "]" << std::endl;
      }

      this->dataPtr->lastCollisionTime = this->dataPtr->currentTime;

      this->dataPtr->collisionList.push_back(
        _contacts.contact(i).collision1().name() +
        std::string(" || ") + _contacts.contact(i).collision2().name());
      this->dataPtr->collisionTimestamps.push_back(
        this->dataPtr->currentTime.count());
      this->OnCollision();
      return;
    }
  }
}

//////////////////////////////////////////////////
void ScoringPlugin::ReleaseVehicle()
{
  ignition::msgs::Empty msg;
  msg.set_unused(true);
  this->dataPtr->releasePub.Publish(msg);
}

IGNITION_ADD_PLUGIN(ScoringPlugin,
                    gazebo::System,
                    ScoringPlugin::ISystemConfigure,
                    ScoringPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(vrx::ScoringPlugin,
                          "vrx::ScoringPlugin")
