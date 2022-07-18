#include <ignition/common/Console.hh>
#include <ignition/physics/Joint.hh>
#include <ignition/plugin/Register.hh>

#include "ScoringPlugin.hh"
using namespace ignition;
using namespace gazebo;
using namespace systems;
void cb(const ignition::msgs::Contacts &_contacts){return;}

ScoringPlugin::ScoringPlugin()
    //: gzNode(new std::unique_ptr<ignition::transport::Node()>)
{
}

void ScoringPlugin::Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr)
{
    // TODO: GZ_ASSERT equivalent (WorldPtr world & ElementPtr sdf)
    // this->world = _world
    this->world = World(_entity); //TODO: verify: Is this how world is accessed?
    this->sdf = _sdf;

    // SDF.
    if (!this->ParseSDFParameters())
    {
        ignerr << "Scoring disabled" << std::endl;
        return;
    }
    // Initialize spherical coordinates
    // auto gzSC = _ecm.Component<components::SphericalCoordinates>(this->world)
    auto gzSC = this->world.SphericalCoordinates(_ecm);

    this->sc.SetLatitudeReference(gzSC->LatitudeReference());
    this->sc.SetLongitudeReference(gzSC->LongitudeReference());
    this->sc.SetElevationReference(gzSC->ElevationReference());
    this->sc.SetHeadingOffset(gzSC->HeadingOffset());

    this->readyTime.set_sec(std::floor(this->initialStateDuration));
    this->readyTime.set_nsec()
    this->runningTime.Set((this->readyTime).Double()+  this->readyStateDuration);
    this->finishTime.Set((this->runningTime).Double() + this->runningStateDuration);

    // Prepopulate the task msg.
    this->taskMsg["name"] = this->taskName;
    this->taskMsg["ready_time"] = builtin_interfaces::msg::Time
                                (rclcpp::Time(this->readyTime.sec,this->readyTime.nsec));
    this->taskMsg["running_time"] = builtin_interfaces::msg::Time
                                (rclcpp::Time(this->runningTime.sec,this->runningTime.nsec));
    this->UpdateTaskMessage();

    // Initialize ROS transport
    // this->taskPub = rosNode->create_publisher<vrx_ros::msg::Task>(this->taskInfoTopic, 100);
    // this->contactPub = rosNode->create_publisher<vrx_ros::msg::Contact>(this->contactDebugTopic, 100);

    this->taskPub = std::make_unique<ignition::transport::Node::Publisher>
        (gzNode->Advertise<ignition::msgs::Param_V>(this->taskInfoTopic));
    this->contactPub = std::make_unique<ignition::transport::Node::Publisher>
        (gzNode->Advertise<ignition::msgs::Contact>(this->contactDebugTopic));

    // TODO: Verify - Is this needed in Ignition?
    // gzNode->Init();

    // TODO: Verify if this is how components of the world are accessed
    // std::string worldName = _ecm.Component<ignition::gazebo::components::Name>(this->world);
    std::string worldName = this->world.Name(_ecm).value();
    
    std::string collisionTopic =
        std::string("/gazebosim/") + worldName + std::string("/physics/contacts");
    if (!gzNode->Subscribe(collisionTopic, &ScoringPlugin::OnCollisionMsg, this))
    {
        std::cerr << "Error subscribing to [" << collisionTopic << "]" << std::endl;
    }

    this->serverControlPub = std::make_unique<ignition::transport::Node::Publisher>
        (gzNode->Advertise<ignition::msgs::ServerControl>
        ("/gazebosim/server/control"));
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
ignition::common::Time ScoringPlugin::ElapsedTime() const
{
  return this->elapsedTime;
}

//////////////////////////////////////////////////
ignition::common::Time ScoringPlugin::RemainingTime() const
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
    // TODO: How to define model ptr (see .hh)? How to access world elements?
    // this->vehicleModel = this->world.vehicleName //??

    this->simTime = _info.simTime;
    this->UpdateTime(simTime);
    this->UpdateTaskState();
    this->PublishStats();
}

//////////////////////////////////////////////////
void ScoringPlugin::UpdateTime(const std::chrono::duration<double> _simTime)
{
    this->currentTime = ignition::common::Time(std::chrono::duration_cast<std::chrono::duration<double> >(_simTime).count());

    this->elapsedTime = this->currentTime - this->runningTime;
    this->remainingTime = this->finishTime - this->currentTime;
    this->timedOut = this->remainingTime <= ignition::common::Time(0,0);
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
    this->taskMsg["state"] = this->taskState;

    this->taskMsg["elapsed_time"] = builtin_interfaces::msg::Duration
                (rclcpp::Duration(this->elapsedTime.sec,this->elapsedTime.nsec));
    this->taskMsg["remaining_time"] = builtin_interfaces::msg::Duration
                (rclcpp::Duration(this->remainingTime.sec,this->remainingTime.nsec));
    this->taskMsg["timed_out"] = this->timedOut;
    this->taskMsg["score"] = this->score;
    this->taskMsg["num_collisions"] = this->numCollisions;
}

//////////////////////////////////////////////////
void ScoringPlugin::PublishStats()
{
  this->UpdateTaskMessage();

  // We publish stats at 1Hz.
  if (this->currentTime - this->lastStatsSent >= ignition::common::Time(1, 0))
  {
    this->taskPub->Publish(this->taskMsg);
    this->lastStatsSent = this->currentTime;
  }
}
//////////////////////////////////////////////////
void ScoringPlugin::ReleaseVehicle()
{
    //TODO: How to access vehicle model?
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
    ignmsg << rosNode->now().seconds() << "  OnFinished" << std::endl;

  // If a timeoutScore was specified, use it.
  if (this->timedOut && this->timeoutScore > 0.0)
  {
    this->score = this->timeoutScore;
  }
  this->UpdateTaskMessage();
  this->taskPub->Publish(this->taskMsg);
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
    //TODO: Implement
    // loop through collisions, if any include the wamv, increment collision
    // counter
    for (unsigned int i = 0; i < _contacts.contact_size(); ++i)
    {
    std::string wamvCollisionStr1 = _contacts.contact(i).collision1().name();
    std::string wamvCollisionStr2 = _contacts.contact(i).collision2().name();
    std::string wamvCollisionSubStr1 =
        wamvCollisionStr1.substr(0, wamvCollisionStr1.find("lump"));
    std::string wamvCollisionSubStr2 =
        wamvCollisionStr2.substr(0, wamvCollisionStr2.find("lump"));

    bool isWamvHit =
        wamvCollisionSubStr1 == "wamv::base_link::base_link_fixed_joint_" ||
        wamvCollisionSubStr1 ==
        "wamv::wamv/base_link::wamv/base_link_fixed_joint_"             ||
        wamvCollisionSubStr2 == "wamv::base_link::base_link_fixed_joint_" ||
        wamvCollisionSubStr2 ==
        "wamv::wamv/base_link::wamv/base_link_fixed_joint_";
    bool isHitBufferPassed = this->currentTime - this->lastCollisionTime >
                                ignition::common::Time(this->collisionBuffer, 0);

    // publish a Contact MSG
    if (isWamvHit && this->debug)
    {
        this->contactMsg.header.stamp = rosNode->now(); //TODO: ignition::msgs::Time
        this->contactMsg.collision1 = _contacts.contact(i).collision1(); //Entity
        this->contactMsg.collision2 = _contacts.contact(i).collision2(); //Entity
        this->contactPub->Publish(this->contactMsg);
    }

    if (isWamvHit && isHitBufferPassed)
    {
        this->collisionCounter++;
        if (!this->silent)
        {
        ignmsg << "[" << this->collisionCounter
                << "] New collision counted between ["
                << _contacts.contact(i).collision1().name() << "] and ["
                << _contacts.contact(i).collision2().name() << "]" << std::endl;
        }
        // Uncomment to get details of collisions
        // igndbg << _contacts.contact(i).DebugString() << std::endl;

        this->lastCollisionTime.Set(std::chrono::duration_cast<std::chrono::duration<double> >(this->simTime).count());
        this->collisionList.push_back(
            _contacts.contact(i).collision1().name() +
            std::string(" || ") + _contacts.contact(i).collision2().name());
        this->collisionTimestamps.push_back(this->currentTime);
        this->OnCollision();
        return;
    }
  }

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
    // shutdown gazebo
    if (rclcpp::ok())
      rclcpp::shutdown();
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