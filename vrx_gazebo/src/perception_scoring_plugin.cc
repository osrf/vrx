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

#include <algorithm>
#include <mutex>
#include <string>
#include <vector>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>
#include "vrx_gazebo/perception_scoring_plugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
ObjectChecker::ObjectChecker(const std::string &_rosNameSpace,
  const std::string &_rosObjectTopic, gazebo::physics::WorldPtr _world)
  : ns(_rosNameSpace),
    objectTopic(_rosObjectTopic),
    world(_world)
{
  // Quit if ros plugin was not loaded
  if (!ros::isInitialized())
  {
    ROS_ERROR("ROS was not initialized.");
    return;
  }

  this->nh = ros::NodeHandle(this->ns);
}

//////////////////////////////////////////////////
void ObjectChecker::NewTrial(const std::string &_objectName,
  gazebo::physics::EntityPtr _object)
{
  // Setup for a new trial
  this->trialCount++;
  this->objectReceived = false;
  this->objectCorrect = false;
  this->submissionScored = false;
  this->objectError = -1.0;

  // Store truth
  this->trueName = _objectName;
  this->currObject = _object;

  ROS_INFO_NAMED("ObjectChecker", "Initiating new trial");
}

//////////////////////////////////////////////////
void ObjectChecker::Enable()
{
  // Subscribe
  this->objectSub = this->nh.subscribe(this->objectTopic, 1,
    &ObjectChecker::OnObject, this);
}

//////////////////////////////////////////////////
void ObjectChecker::Disable()
{
  this->objectSub.shutdown();
}

//////////////////////////////////////////////////
bool ObjectChecker::SubmissionReceived() const
{
  return this->objectReceived;
}

//////////////////////////////////////////////////
bool ObjectChecker::Correct() const
{
  return this->objectCorrect;
}

//////////////////////////////////////////////////
void ObjectChecker::OnObject(
  const geographic_msgs::GeoPoseStamped::ConstPtr &_msg)
{
  // Only accept one message per trial
  if (this->objectReceived)
  {
    ROS_WARN_NAMED("ObjectChecker", "Receiving multiple ID messages for same "
      "trial.  Ignoring.");
    return;
  }

  // Accept the message
  this->objectReceived = true;
  this->objectCorrect = !this->trueName.compare(_msg->header.frame_id);
  // Convert geo pose to Gazebo pose
  // Note - this is used in a few different VRX plugins, may want to have a
  // separate library?
  // Convert lat/lon to local
  // Snippet from UUV Simulator SphericalCoordinatesROSInterfacePlugin.cc
  ignition::math::Vector3d scVec(_msg->pose.position.latitude,
    _msg->pose.position.longitude, 0);
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Vector3d cartVec =
  this->world->SphericalCoords()->LocalFromSpherical(scVec);
#else
  ignition::math::Vector3d cartVec =
  this->world->GetSphericalCoordinates()->LocalFromSpherical(scVec);
#endif

  // Get current pose of the current object
  #if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d truePose = this->currObject->WorldPose();
  #else
    ignition::math::Pose3d truePose = this->currObject->GetWorldPose().Ign();
  #endif

  // 2D Error
  this->objectError = sqrt(pow(cartVec.X() - truePose.Pos().X(), 2)+
    pow(cartVec.Y() - truePose.Pos().Y(), 2));

  ROS_INFO_NAMED("ObjectChecker", "Object ID: true = %s, submitted = %s "
    "result=%d; 2D position error = %.3f m",
    this->trueName.c_str(), _msg->header.frame_id.c_str(),
    static_cast<int>(this->objectCorrect), this->objectError);
}

//////////////////////////////////////////////////
/// \internal
/// \brief Private data for the PerceptionScoringPlugin class.
struct PerceptionScoringPluginPrivate
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

    /// \brief Object type.
    public: std::string name;

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

  /// \brief Link/model name for the object poses use as their frame of
  /// reference
  public: std::string frameName = std::string();

  /// \brief Link/model that the object poses use as their frame of reference.
  public: physics::EntityPtr frame;

  /// \brief Current object that has been placed
  public: physics::EntityPtr curr_model;

  /// \brief Current object's original pose
  public: ignition::math::Pose3d orig_pose;

  /// \brief Mutex to avoid race conditions.
  public: std::mutex mutex;

  /// \brief Last time (sim time) that the plugin was updated.
  public: gazebo::common::Time lastUpdateTime;

  /// \brief Counter for spawning objects with unique names on the belt.
  /// The key is the object type and the value contains the index of the next
  /// object to be spawned.
  public: std::map<std::string, int> objectCounter;

  /// \brief Implements ROS interface to recieve and check team submissions
  public: std::unique_ptr<ObjectChecker> objectChecker;
};

GZ_REGISTER_WORLD_PLUGIN(PerceptionScoringPlugin)

/////////////////////////////////////////////////
PerceptionScoringPlugin::PerceptionScoringPlugin()
  : dataPtr(new PerceptionScoringPluginPrivate)
{
  gzmsg << "PerceptionScoringPlugin loaded" << std::endl;
}

/////////////////////////////////////////////////
PerceptionScoringPlugin::~PerceptionScoringPlugin()
{
}

/////////////////////////////////////////////////
void PerceptionScoringPlugin::Load(physics::WorldPtr _world,
  sdf::ElementPtr _sdf)
{
  // Base class, also binds the update method for the base class
  ScoringPlugin::Load(_world, _sdf);

  this->dataPtr->world = _world;
  this->dataPtr->sdf = _sdf;

  // Read the SDF parameters
  if (_sdf->HasElement("loop_forever"))
  {
    sdf::ElementPtr loopElem = _sdf->GetElement("loop_forever");
    this->dataPtr->loopForever = loopElem->Get<bool>();
  }

  if (_sdf->HasElement("frame"))
  {
    this->dataPtr->frameName = _sdf->Get<std::string>("frame");
  }

  if (!_sdf->HasElement("object_sequence"))
  {
    gzerr << "PerceptionScoringPlugin: Unable to find <object_sequence> "
      "element\n";
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
      gzerr << "PerceptionScoringPlugin: Unable to find <time> in object\n";
      objectElem = objectElem->GetNextElement("object");
      continue;
    }
    sdf::ElementPtr timeElement = objectElem->GetElement("time");
    double time = timeElement->Get<double>();

    // Parse the object type.
    if (!objectElem->HasElement("type"))
    {
      gzerr << "PerceptionScoringPlugin: Unable to find <type> in object.\n";
      objectElem = objectElem->GetNextElement("object");
      continue;
    }
    sdf::ElementPtr typeElement = objectElem->GetElement("type");
    std::string type = typeElement->Get<std::string>();

    // Parse the object name - this is what must be matched id success.
    if (!objectElem->HasElement("name"))
    {
      gzerr << "PerceptionScoringPlugin: Unable to find <name> in object.\n";
      objectElem = objectElem->GetNextElement("object");
      continue;
    }
    sdf::ElementPtr nameElement = objectElem->GetElement("name");
    std::string name = nameElement->Get<std::string>();

    // Parse the object pose
    if (!objectElem->HasElement("pose"))
    {
      gzerr << "PerceptionScoringPlugin: Unable to find <pose> in object.\n";
        objectElem = objectElem->GetNextElement("object");
      continue;
    }
    sdf::ElementPtr poseElement = objectElem->GetElement("pose");
    ignition::math::Pose3d pose = poseElement->Get<ignition::math::Pose3d>();

    // Add the object to the collection.
    PerceptionScoringPluginPrivate::Object obj = {time, type, name, pose};
    this->dataPtr->initialObjects.push_back(obj);

    objectElem = objectElem->GetNextElement("object");
  }

  std::sort(this->dataPtr->initialObjects.begin(),
    this->dataPtr->initialObjects.end());
  #if GAZEBO_MAJOR_VERSION >= 8
    this->dataPtr->lastUpdateTime = this->dataPtr->world->SimTime();
  #else
    this->dataPtr->lastUpdateTime = this->dataPtr->world->GetSimTime();
  #endif

  // Optional: ROS namespace.
  std::string ns;
  if (_sdf->HasElement("robot_namespace"))
    ns = _sdf->GetElement("robot_namespace")->Get<std::string>();

  // Optional: ROS topic.
  std::string object_topic = "/vrx/perception/landmark";
  if (_sdf->HasElement("landmark_topic"))
  {
    object_topic = _sdf->GetElement("landmark_topic")->Get<std::string>();
  }
  // Instatiate the object checker
  this->dataPtr->objectChecker.reset(
    new ObjectChecker(ns, object_topic, _world));

  this->Restart();

  this->dataPtr->connection = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&PerceptionScoringPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void PerceptionScoringPlugin::Restart()
{
  #if GAZEBO_MAJOR_VERSION >= 8
    this->dataPtr->startTime = this->dataPtr->world->SimTime();
  #else
    this->dataPtr->startTime = this->dataPtr->world->GetSimTime();
  #endif

  this->dataPtr->objects = this->dataPtr->initialObjects;

  // gzmsg << "Object population restarted" << std::endl;
}

/////////////////////////////////////////////////
void PerceptionScoringPlugin::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Connect with object checker to see if we need to score a submission
  if (this->dataPtr->objectChecker->SubmissionReceived() &&
      !this->dataPtr->objectChecker->submissionScored)
  {
    ROS_INFO("Scoring a new submission");
    // Only add score if the identification is correct
    if (this->dataPtr->objectChecker->Correct())
    {
      this->SetScore(this->Score()+this->dataPtr->objectChecker->objectError);
    }
    // Mark trial as scored
    this->dataPtr->objectChecker->submissionScored = true;
  }

  // Have we completed a cycle through the object queue?
  if (this->dataPtr->objects.empty())
  {
    if (this->dataPtr->loopForever)
    {
      this->Restart();
    }
    else
    {
      #if GAZEBO_MAJOR_VERSION >= 8
        this->dataPtr->lastUpdateTime = this->dataPtr->world->SimTime();
      #else
        this->dataPtr->lastUpdateTime = this->dataPtr->world->GetSimTime();
      #endif
      return;
    }
  }

  // Check whether move the next object in the list.
  if (this->ElapsedTime() >= this->dataPtr->objects.front().time)
  {
    gzmsg << "PerceptionScoringPlugin: spawn next object." << std::endl;
    auto obj = this->dataPtr->objects.front();

    // Try to use a model/link frame if specified.
    if (!this->dataPtr->frameName.empty())
    {
      #if GAZEBO_MAJOR_VERSION >= 8
        this->dataPtr->frame =
          this->dataPtr->world->EntityByName(this->dataPtr->frameName);
      #else
        this->dataPtr->frame =
          this->dataPtr->world->GetEntity(this->dataPtr->frameName);
      #endif
      if (!this->dataPtr->frame)
      {
        gzthrow(std::string("The frame '") + this->dataPtr->frameName +
          "' does not exist");
      }
      if (!this->dataPtr->frame->HasType(physics::Base::LINK) &&
          !this->dataPtr->frame->HasType(physics::Base::MODEL))
      {
        gzthrow("'frame' tag must list the name of a link or model");
      }
    }

    if (this->dataPtr->frame)
    {
      // Set object pose relative to the specified frame (e.g., the wam-v)
      // Pitch and roll are set to zero as a hack to deal with
      // transients associated with spawning buoys with significant attitude.
      #if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d framePose(
          this->dataPtr->frame->WorldPose().Pos(),
          ignition::math::Quaterniond(0.0, 0.0,
            this->dataPtr->frame->WorldPose().Rot().Yaw()));
      #else
        ignition::math::Pose3d framePose(
          this->dataPtr->frame->GetWorldPose().pos.Ign(),
          ignition::math::Quaterniond(0.0, 0.0,
            this->dataPtr->frame->GetWorldPose().rot.Ign().Yaw()));
      #endif
      ignition::math::Matrix4d transMat(framePose);
      ignition::math::Matrix4d pose_local(obj.pose);
      obj.pose = (transMat * pose_local).Pose();
    }

    std::string modelName = obj.type;

    // Get a new index for the object.
    if (this->dataPtr->objectCounter.find(obj.type) ==
        this->dataPtr->objectCounter.end())
    {
    this->dataPtr->objectCounter[obj.type] = 0;
    }
    else
    {
      this->dataPtr->objectCounter[obj.type]++;
    }
    int index = this->dataPtr->objectCounter[obj.type];

    // If necessary, move object back to original pose
    if (this->dataPtr->curr_model)
    {
      this->dataPtr->curr_model->SetWorldPose(this->dataPtr->orig_pose);
    }
    // Get a unique name for the new object.
    modelName += "_" + std::to_string(index);
    #if GAZEBO_MAJOR_VERSION >= 8
      auto modelPtr = this->dataPtr->world->EntityByName(modelName);
    #else
      auto modelPtr = this->dataPtr->world->GetEntity(modelName);
    #endif
    if (modelPtr)
    {
      // Setup new trial in object checker
      this->dataPtr->objectChecker->NewTrial(obj.name, modelPtr);

      // Save current object and original pose for later
      this->dataPtr->curr_model = modelPtr;
      #if GAZEBO_MAJOR_VERSION >= 8
        this->dataPtr->orig_pose = modelPtr->WorldPose();
      #else
        this->dataPtr->orig_pose = modelPtr->GetWorldPose().Ign();
      #endif
      // Move object to the target pose.
      modelPtr->SetWorldPose(obj.pose);
      modelPtr->SetWorldTwist(ignition::math::Vector3d::Zero,
        ignition::math::Vector3d::Zero);
      gzdbg << "Object [" << modelName << "] spawned" << std::endl;
    }
    else
    {
      gzerr << "Object [" << modelName << "] NOT spawned" << std::endl;
    }
    this->dataPtr->objects.erase(this->dataPtr->objects.begin());
    #if GAZEBO_MAJOR_VERSION >= 8
      this->dataPtr->lastUpdateTime = this->dataPtr->world->SimTime();
    #else
      this->dataPtr->lastUpdateTime = this->dataPtr->world->GetSimTime();
    #endif
  }
}

//////////////////////////////////////////////////
void PerceptionScoringPlugin::OnRunning()
{
  gzmsg << "OnRunning" << std::endl;

  this->dataPtr->objectChecker->Enable();
}
