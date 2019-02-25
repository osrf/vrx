/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <algorithm>
#include <mutex>
#include <ostream>
#include <string>
#include <vector>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/gz_string.pb.h>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Pose3.hh>
#include <sdf/sdf.hh>

#include "osrf_gear/PopulationPlugin.hh"

namespace gazebo
{
  /// \internal
  /// \brief Private data for the PopulationPlugin class.
  struct PopulationPluginPrivate
  {
    /// \brief World pointer.
    public: physics::WorldPtr world;

    /// \brief SDF pointer.
    public: sdf::ElementPtr sdf;

    /// \brief Class to store information about each object to be populated.
    public: class Object
            {
             /// \brief Less than operator.
             /// \param[in] _obj Other object to compare
             /// \return True if this < _obj
             public: bool operator<(const Object &_obj) const
             {
               return this->time < _obj.time;
             }

             /// \brief Stream insertion operator.
             /// \param[in] _out output stream
             /// \param[in] _obj object to output
             /// \return The output stream
             public: friend std::ostream &operator<<(std::ostream &_out,
                                                     const Object &_obj)
             {
               _out << _obj.type << std::endl;
               _out << "  Time: [" << _obj.time << "]" << std::endl;
               _out << "  Pose: [" << _obj.pose << "]" << std::endl;
               return _out;
             }

              /// \brief Simulation time in which the object should be spawned.
              public: double time;

              /// \brief Object type.
              public: std::string type;

              /// \brief Pose in which the object should be placed.
              public: ignition::math::Pose3d pose;
            };

    /// \brief Collection of objects to be spawned.
    public: std::vector<Object> objects;

    /// \brief Contains the entire collection of objects. This is used for
    /// inserting the objects in a cyclic way.
    public: std::vector<Object> initialObjects;

    /// \brief Connection event.
    public: event::ConnectionPtr connection;

    /// \brief The time specified in the object is relative to this time.
    public: common::Time startTime;

    /// \brief When true, "objects" will be repopulated when the object queue
    /// is empty, creating an infinite supply of objects.
    public: bool loopForever = false;

    /// \brief Link/model that the object poses use as their frame of reference.
    public: physics::EntityPtr frame;

    /// \brief Node for communication.
    public: transport::NodePtr node;

    /// \brief Subscriber to the activation topic.
    public: transport::SubscriberPtr activationSub;

    /// \brief Subscriber to the rate modifier topic.
    public: transport::SubscriberPtr rateModifierSub;

    /// \brief If true, the objects will start populating.
    public: bool enabled = false;

    /// \brief Mutex to avoid race conditions.
    public: std::mutex mutex;

    /// \brief Elapsed time since "startTime" when the plugin is paused.
    public: common::Time elapsedWhenPaused;

    /// \brief Plugin update rate (Hz). A negative value means that the custom
    /// update rate is disabled. The plugin will execute at the physics rate.
    public: double updateRate = -1;

    /// \brief Equivalent time since last object spawning, taking rate modifier into account.
    public: double elapsedEquivalentTime = 0.0;

    /// \brief Rate modifier of the populating: 1.0 will populate at the standard rate,
    /// other values will scale the populating frequency.
    public: double rateModifier;

    /// \brief Object names will be prefixed by plugin name if True.
    public: bool prefixObjectNames = true;

    /// \brief Id of first object to teleport.
    public: int startIndex = 0;

    /// \brief Last time (sim time) that the plugin was updated.
    public: gazebo::common::Time lastUpdateTime;

    /// \brief Counter for spawning objects with unique names on the belt.
    /// The key is the object type and the value contains the index of the next
    /// object to be spawned.
    public: std::map<std::string, int> objectCounter;


  };
}

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(PopulationPlugin)

/////////////////////////////////////////////////
PopulationPlugin::PopulationPlugin()
  : dataPtr(new PopulationPluginPrivate)
{
}

/////////////////////////////////////////////////
PopulationPlugin::~PopulationPlugin()
{
}

/////////////////////////////////////////////////
void PopulationPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "PopulationPlugin world pointer is NULL");
  GZ_ASSERT(_sdf, "PopulationPlugin sdf pointer is NULL");
  this->dataPtr->world = _world;
  this->dataPtr->sdf = _sdf;

  // Read the SDF parameters
  if (_sdf->HasElement("loop_forever"))
  {
    sdf::ElementPtr loopElem = _sdf->GetElement("loop_forever");
    this->dataPtr->loopForever = loopElem->Get<bool>();
  }

  if (_sdf->HasElement("start_index"))
  {
    this->dataPtr->startIndex = _sdf->Get<int>("start_index");
  }

  if (_sdf->HasElement("prefix_object_names"))
  {
    this->dataPtr->prefixObjectNames = _sdf->Get<bool>("prefix_object_names");
  }

  if (_sdf->HasElement("frame"))
  {
    std::string frameName = _sdf->Get<std::string>("frame");
    this->dataPtr->frame = this->dataPtr->world->EntityByName(frameName);
    if (!this->dataPtr->frame) {
      gzthrow(std::string("The frame '") + frameName + "' does not exist");
    }
    if (!this->dataPtr->frame->HasType(physics::Base::LINK) &&
      !this->dataPtr->frame->HasType(physics::Base::MODEL))
    {
      gzthrow("'frame' tag must list the name of a link or model");
    }
  }

  if (!_sdf->HasElement("object_sequence"))
  {
    gzerr << "PopulationPlugin: Unable to find <object_sequence> element\n";
    return;
  }

  sdf::ElementPtr sequence = _sdf->GetElement("object_sequence");

  sdf::ElementPtr objectElem = NULL;
  if (sequence->HasElement("object"))
  {
    objectElem = sequence->GetElement("object");
  }

  while (objectElem)
  {
    // Parse the time.
    if (!objectElem->HasElement("time"))
    {
      gzerr << "PopulationPlugin: Unable to find <time> in object\n";
      objectElem = objectElem->GetNextElement("object");
      continue;
    }
    sdf::ElementPtr timeElement = objectElem->GetElement("time");
    double time = timeElement->Get<double>();

    // Parse the object type.
    if (!objectElem->HasElement("type"))
    {
      gzerr << "PopulationPlugin: Unable to find <type> in object.\n";
      objectElem = objectElem->GetNextElement("object");
      continue;
    }
    sdf::ElementPtr typeElement = objectElem->GetElement("type");
    std::string type = typeElement->Get<std::string>();

    // Parse the object pose (optional).
    ignition::math::Pose3d pose;
    if (objectElem->HasElement("pose"))
    {
      sdf::ElementPtr poseElement = objectElem->GetElement("pose");
      pose = poseElement->Get<ignition::math::Pose3d>();
    }

    // Add the object to the collection.
    PopulationPluginPrivate::Object obj = {time, type, pose};
    this->dataPtr->initialObjects.push_back(obj);

    objectElem = objectElem->GetNextElement("object");
  }
  std::sort(this->dataPtr->initialObjects.begin(),
    this->dataPtr->initialObjects.end());

  // Create and initialize the node.
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  // Listen on the activation topic, if present. This topic is used for
  // manual activation.
  if (_sdf->HasElement("activation_topic"))
  {
    // Subscribe to the activation topic.
    this->dataPtr->activationSub = this->dataPtr->node->Subscribe(
        _sdf->Get<std::string>("activation_topic"),
        &PopulationPlugin::OnActivation, this);
  }
  else
    this->Restart();

  this->dataPtr->lastUpdateTime = this->dataPtr->world->SimTime();

  // Listen on the activation topic, if present. This topic is used for
  // manual activation.
  if (_sdf->HasElement("rate_modifier_topic"))
  {
    // Subscribe to the rate modifier topic.
    this->dataPtr->rateModifierSub = this->dataPtr->node->Subscribe(
        _sdf->Get<std::string>("rate_modifier_topic"),
        &PopulationPlugin::OnRateModification, this);
    this->dataPtr->rateModifier = 0.0;
  }
  else
  {
    this->dataPtr->rateModifier = 1.0;
  }

  this->dataPtr->connection = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&PopulationPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void PopulationPlugin::Pause()
{
  if (!this->dataPtr->enabled)
    return;

  this->dataPtr->enabled = false;
  this->dataPtr->elapsedWhenPaused =
    this->dataPtr->world->SimTime() - this->dataPtr->startTime;

  gzmsg << "Object population paused" << std::endl;
}

/////////////////////////////////////////////////
void PopulationPlugin::Resume()
{
  if (this->dataPtr->enabled)
    return;

  this->dataPtr->enabled = true;
  this->dataPtr->startTime = this->dataPtr->world->SimTime() -
    this->dataPtr->elapsedWhenPaused;

   gzmsg << "Object population resumed" << std::endl;
}

/////////////////////////////////////////////////
void PopulationPlugin::Restart()
{
  this->dataPtr->enabled = true;
  this->dataPtr->elapsedEquivalentTime = 0;
  this->dataPtr->startTime = this->dataPtr->world->SimTime();
  this->dataPtr->objects = this->dataPtr->initialObjects;

  // gzmsg << "Object population restarted" << std::endl;
}

/////////////////////////////////////////////////
void PopulationPlugin::OnUpdate()
{
  // If we're using a custom update rate value we have to check if it's time to
  // update the plugin or not.
  if (!this->TimeToExecute())
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->Publish();

  if (!this->dataPtr->enabled)
  {
    this->dataPtr->lastUpdateTime = this->dataPtr->world->SimTime();
    return;
  }

  if (this->dataPtr->objects.empty())
  {
    if (this->dataPtr->loopForever)
    {
      this->Restart();
    }
    else
    {
      this->dataPtr->lastUpdateTime = this->dataPtr->world->SimTime();
      return;
    }
  }

  // Check whether spawn a new object from the list.
  auto elapsedTime = this->dataPtr->world->SimTime() - this->dataPtr->lastUpdateTime;
  // The rate modifier has the effect of slowing down sim time in this plugin.
  // If the rate modifier is 0.5, we will wait for twice as much time before spawning a new object.
  this->dataPtr->elapsedEquivalentTime += elapsedTime.Double() * this->dataPtr->rateModifier;
  if (this->dataPtr->elapsedEquivalentTime >= this->dataPtr->objects.front().time)
  {
    auto obj = this->dataPtr->objects.front();
    if (this->dataPtr->frame)
    {
      auto framePose = this->dataPtr->frame->WorldPose();
      ignition::math::Matrix4d transMat(framePose);
      ignition::math::Matrix4d pose_local(obj.pose);
      obj.pose = (transMat * pose_local).Pose();
    }

    std::string modelName = obj.type;
    if (this->dataPtr->prefixObjectNames)
    {
      modelName = this->GetHandle() + "|" + modelName;
    }

    // Get a new index for the object.
    if (this->dataPtr->objectCounter.find(obj.type) ==
        this->dataPtr->objectCounter.end())
    {
      this->dataPtr->objectCounter[obj.type] = this->dataPtr->startIndex;
    }
    else
    {
      this->dataPtr->objectCounter[obj.type]++;
    }
    int index = this->dataPtr->objectCounter[obj.type];

    // Get a unique name for the object.
    modelName += "_" + std::to_string(index);
    auto modelPtr = this->dataPtr->world->ModelByName(modelName);
    if (modelPtr)
    {
      // Move it to the target pose.
      modelPtr->SetWorldPose(obj.pose);
      modelPtr->SetLinearVel(ignition::math::Vector3d::Zero);
      modelPtr->SetLinearAccel(ignition::math::Vector3d::Zero);
      gzdbg << "Object [" << modelName << "] on belt" << std::endl;
    }

    this->dataPtr->objects.erase(this->dataPtr->objects.begin());
    this->dataPtr->elapsedEquivalentTime = 0.0;
  }
    this->dataPtr->lastUpdateTime = this->dataPtr->world->SimTime();
}

/////////////////////////////////////////////////
void PopulationPlugin::OnActivation(ConstGzStringPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  gzdbg << "PopulationPlugin: received activation request: " << _msg->data() << std::endl;

  if (_msg->data() == "restart")
    this->Restart();
  else if (_msg->data() == "pause")
    this->Pause();
  else if (_msg->data() == "resume")
    this->Resume();
  else
    gzerr << "Unknown activation command [" << _msg->data() << "]" << std::endl;
}

/////////////////////////////////////////////////
void PopulationPlugin::OnRateModification(ConstGzStringPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  gzdbg << "PopulationPlugin: received rate modification request: " << _msg->data() << std::endl;

  double rateModifier = std::stod(_msg->data());
  if (rateModifier >= 0.0)
  {
    this->dataPtr->rateModifier = rateModifier;
  }
  else
  {
    gzdbg << "Ignoring rate modification request with negative value: " << _msg->data() << std::endl;
  }
}

/////////////////////////////////////////////////
bool PopulationPlugin::Enabled() const
{
  return this->dataPtr->enabled;
}

/////////////////////////////////////////////////
void PopulationPlugin::Publish() const
{
}

/////////////////////////////////////////////////
bool PopulationPlugin::TimeToExecute()
{
  gazebo::common::Time curTime = this->dataPtr->world->SimTime();
  // We're using a custom update rate.
  if (this->dataPtr->updateRate <= 0)
    return true;

  auto dt = (curTime - this->dataPtr->lastUpdateTime).Double();
  if (dt < 0)
  {
    // Probably we had a reset.
    this->dataPtr->lastUpdateTime = curTime;
    return false;
  }

  // Update based on sensorsUpdateRate.
  if (dt < (1.0 / this->dataPtr->updateRate))
    return false;

  return true;
}
