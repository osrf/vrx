#include <ignition/common/Console.hh>
#include <ignition/physics/Joint.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Events.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>
#include "ScoringPlugin.hh"

void cb(const ignition::msgs::Contacts &_contacts){return;}

ScoringPlugin::ScoringPlugin()

{
}

void ScoringPlugin::Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr)
{

    this->creator = 
        std::make_unique<ignition::gazebo::SdfEntityCreator>(_ecm, _eventMgr);
    this->worldEntity = 
        _ecm.EntityByComponents(ignition::gazebo::components::World());
    this->eventManager = &_eventMgr;
    this->sdf = _sdf;
    //this->forceReturn = true; // Turns off update function! Remove before use.
    // SDF.
    if (!this->ParseSDFParameters())
    {
        ignerr << "Scoring disabled" << std::endl;

        return;
    }

    this->readyTime = std::chrono::duration<double>(this->initialStateDuration);
    this->runningTime = this->readyTime +  
        std::chrono::duration<double>(this->readyStateDuration);
    this->finishTime = this->runningTime + 
        std::chrono::duration<double>(this->runningStateDuration);

    // Prepopulate the task msg.

  taskMessage.set_data(this->taskState);

  this->taskPub = 
        this->node.Advertise<ignition::msgs::StringMsg>(this->taskInfoTopic);
  this->contactPub = 
        this->node.Advertise<ignition::msgs::Contact>(this->contactDebugTopic);


}

//////////////////////////////////////////////////
double ScoringPlugin::Score() const
{
  return this->score;
}

//////////////////////////////////////////////////
void ScoringPlugin::SetScore(double _newScore)
{
  if (this->TaskState() == "running")
    this->score = _newScore;
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
std::chrono::duration<double> ScoringPlugin::ElapsedTime() const
{
  return this->elapsedTime;
}

//////////////////////////////////////////////////
std::chrono::duration<double> ScoringPlugin::RemainingTime() const
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
void ScoringPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm)
{
    // Placeholder for testing
    if (this->forceReturn)
    {
      return;
    }

    this->simTime = _info.simTime;
    this->UpdateTime(simTime);
    this->UpdateTaskState();
    this->PublishStats();
}

//////////////////////////////////////////////////
void ScoringPlugin::UpdateTime(const std::chrono::duration<double> _simTime)
{
    this->currentTime = _simTime;
    this->elapsedTime = this->currentTime - this->runningTime;
    this->remainingTime = this->finishTime - this->currentTime;
    this->timedOut = this->remainingTime <= std::chrono::duration<double>(0.0);
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

void ScoringPlugin::UpdateTaskMessage() //My simplified version
{
    this->taskMessage.set_data(this->taskState);
}
//////////////////////////////////////////////////
void ScoringPlugin::PublishStats()
{
  this->UpdateTaskMessage();

  // We publish stats at 1Hz.
  if (this->currentTime - this->lastStatsSent >= 
      std::chrono::duration<double>(1.0))
  {
    this->taskPub.Publish(this->taskMessage);
    this->lastStatsSent = this->currentTime;
  }
}
//////////////////////////////////////////////////
void ScoringPlugin::ReleaseVehicle()
{
    ignmsg << "Scoring Plugin Released WAMV" << std::endl;
}

void ScoringPlugin::OnReady()
{
  if (!this->silent)
    ignmsg << "ScoringPlugin::OnReady" << std::endl;
}

//////////////////////////////////////////////////
void ScoringPlugin::OnRunning()
{
  if (!this->silent)
    ignmsg << "ScoringPlugin::OnRunning" << std::endl;
}

//////////////////////////////////////////////////
void ScoringPlugin::OnFinished()
{
  if (!this->silent)
    ignmsg << this->simTime.count() << "  OnFinished" << std::endl;

  // If a timeoutScore was specified, use it.
  if (this->timedOut && this->timeoutScore > 0.0)
  {
    this->score = this->timeoutScore;
  }
  this->UpdateTaskMessage();
  this->taskPub.Publish(this->taskMessage);
  this->Exit();
}

//////////////////////////////////////////////////
void ScoringPlugin::OnCollision()
{
  ++this->numCollisions;
}


//////////////////////////////////////////////////
void ScoringPlugin::OnCollisionMsg(const ignition::msgs::Contacts &_contacts)
{
    // loop through collisions, if any include the wamv, increment collision
    // counter
  ignmsg << "Collision Message" << std::endl;

}

//////////////////////////////////////////////////
bool ScoringPlugin::ParseSDFParameters()
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
  if (this->sdf->HasElement("contact_debug_topic"))
    this->contactDebugTopic = this->sdf->Get<std::string>
      ("contact_debug_topic");

  // This is an optional element.
  if (this->sdf->HasElement("per_plugin_exit_on_completion"))
    this->perPluginExitOnCompletion = this->sdf->Get<bool>(
      "per_plugin_exit_on_completion");

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
  if (this->sdf->HasElement("collision_buffer"))
  {
    this->collisionBuffer = this->sdf->Get<float>("collision_buffer");
  }

  // This is an optional element.
  if (this->sdf->HasElement("silent"))
  {
    this->silent = this->sdf->Get<bool>("silent");
  }

  return this->ParseJoints();
}

//////////////////////////////////////////////////
bool ScoringPlugin::ParseJoints()
{   
  auto sdf = const_cast<sdf::Element *>(this->sdf.get());
  // Optional element.
  if (this->sdf->HasElement("release_joints"))
  {
    const sdf::ElementPtr releaseJointsElem = sdf->GetElement("release_joints");

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

void ScoringPlugin::Exit()
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
    ignition::msgs::ServerControl msg;
    msg.set_stop(true);
    this->serverControlPub->Publish(msg);
  }
  else
  {
    ignerr << "VRX_EXIT_ON_COMPLETION and <per_plugin_exit_on_completion> "
      << "both not set, will not shutdown on ScoringPlugin::Exit()"
      << std::endl;
  }
  return;
}

void ScoringPlugin::SetTimeoutScore(double _timeoutScore)
{
  this->timeoutScore = _timeoutScore;
}

double ScoringPlugin::GetTimeoutScore() const
{
  return this->timeoutScore;
}

double ScoringPlugin::GetRunningStateDuration() const
{
  return this->runningStateDuration;
}

unsigned int ScoringPlugin::GetNumCollisions() const
{
  return this->numCollisions;
}


IGNITION_ADD_PLUGIN(ScoringPlugin,
                    ignition::gazebo::System,
                    ScoringPlugin::ISystemConfigure,
                    ScoringPlugin::ISystemPostUpdate)