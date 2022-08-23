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

#include <chrono>
#include <string>
#include <vector>
#include <ignition/common/Profiler.hh>
#include <ignition/math/SphericalCoordinates.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/transport/Node.hh>
#include <ignition/plugin/Register.hh>
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

  /// \brief Parse the joints section of the SDF block.
  /// \return True when all parameters were successfully parsed or false
  /// otherwise.
  public: bool ParseJoints();

  /// \brief The name of the task.
  public: std::string taskName = "undefined";

  /// \brief The name of the vehicle to score.
  public: std::string vehicleName;

  /// \brief Vehicle to score.
  public: gazebo::Model vehicleModel;

  /// \brief Silent mode enabled?
  public: bool silent = false;

  /// \brief Spherical coordinates conversions.
  public: math::SphericalCoordinates sc;

  /// \brief gazebo node pointer
  public: transport::Node node;

  /// \brief Topic where the task stats are published.
  public: std::string taskInfoTopic = "/vrx/task/info";

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

  /// \brief Whether the current task has timed out or not.
  public: bool timedOut = false;

  /// \brief Time at which the last message was sent.
  public: std::chrono::duration<double> lastStatsSent{0};

  /// \brief The task state.
  public: std::string taskState = "initial";

  /// \brief The name of the joints to be dettached during ReleaseVehicle().
  public: std::vector<std::string> lockJointNames;

  /// \brief Score in case of timeout - added for Navigation task
  public: double timeoutScore = -1;

  /// \brief Whether to shut down after last gate is crossed.
  public: bool perPluginExitOnCompletion = true;
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
  if (this->sdf->HasElement("silent"))
  {
    this->silent = this->sdf->Get<bool>("silent");
  }

  return this->ParseJoints();
}

//////////////////////////////////////////////////
bool ScoringPlugin::Implementation::ParseJoints()
{
  // Optional element.
  if (this->sdf->HasElement("release_joints"))
  {
    auto releaseJointsElem = this->sdf->GetElement("release_joints");

    // We need at least one joint.
    if (!releaseJointsElem->HasElement("joint"))
    {
      ignerr << "Unable to find <joint> element in SDF." << std::endl;
      return false;
    }

    auto jointElem = releaseJointsElem->GetElement("joint");

    // Parse a new joint to be released.
    while (jointElem)
    {
      // The joint's name.
      if (!jointElem->HasElement("name"))
      {
        ignerr << "Unable to find <name> element in SDF." << std::endl;
        return false;
      }

      const std::string jointName = jointElem->Get<std::string>("name");
      this->lockJointNames.push_back(jointName);

      // Parse the next joint.
      jointElem = jointElem->GetNextElement("joint");
    }
  }

  return true;
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
    gazebo::EventManager &/*_eventMgr*/)
{
  this->dataPtr->sdf = _sdf->Clone();

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

  if (char* envDbg = std::getenv("VRX_DEBUG"))
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
}

IGNITION_ADD_PLUGIN(ScoringPlugin,
                    gazebo::System,
                    ScoringPlugin::ISystemConfigure,
                    ScoringPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(vrx::ScoringPlugin,
                          "vrx::ScoringPlugin")
