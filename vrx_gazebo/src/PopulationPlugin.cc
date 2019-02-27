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
#include <gazebo/transport/transport.hh>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Pose3.hh>
#include <sdf/sdf.hh>
#include "vrx_gazebo/PopulationPlugin.hh"

//////////////////////////////////////////////////
ObjectChecker::ObjectChecker(const std::string &_rosNameSpace,
							 const std::string &_rosObjectTopic,
							 gazebo::physics::WorldPtr _world)
	:ns(_rosNameSpace),
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

void ObjectChecker::NewTrial(const std::string &_objectName,
				   ignition::math::Pose3d _pose)
{
	// Setup for a new trial
	this->trialCount++;
	this->objectReceived = false;
	this->objectCorrect = false;
	this->objectError = -1.0;

	// Store truth
	this->trueName = _objectName;
	this->truePose = _pose;

	ROS_INFO_NAMED("ObjectChecker","Intiating new trial");
}

void ObjectChecker::Enable()
{
	// Subscribe
	this->objectSub = this->nh.subscribe(this->objectTopic,1,
										 &ObjectChecker::OnObject,
										 this);
}

void ObjectChecker::Disable()
{
	this->objectSub.shutdown();
}

bool ObjectChecker::SubmissionReceived() const
{
	return this->objectReceived;
}

bool ObjectChecker::Correct() const
{
	return this->objectCorrect;
}

void ObjectChecker::OnObject(const geographic_msgs::GeoPoseStamped::ConstPtr &_msg)
{
	// Only accept one message per trial
	if (this->objectReceived)
	{
		ROS_WARN_NAMED("ObjectChecker","Receiving multiple ID messages for same trial.  Ignoring.");
		return;
	}
	
	// Accept the message
	this->objectReceived = true;
	this->objectCorrect = !this->trueName.compare(_msg->header.frame_id);
	// Convert geo pose to Gazebo pose
	// Note - this is used in a few different VRX plugins, may want to have a separate library?
	// Convert lat/lon to local
	// Snippet from UUV Simulator SphericalCoordinatesROSInterfacePlugin.cc
	ignition::math::Vector3d scVec(_msg->pose.position.latitude,
								   _msg->pose.position.longitude,
								   0);
    #if GAZEBO_MAJOR_VERSION >= 8
	  ignition::math::Vector3d cartVec =
		  this->world->SphericalCoords()->LocalFromSpherical(scVec);
    #else
	  ignition::math::Vector3d cartVec =
		  this->world->GetSphericalCoordinates()->LocalFromSpherical(scVec);
    #endif
	
	  this->objectError = sqrt(pow(cartVec.X() - this->truePose.Pos().X(),2)+
							   pow(cartVec.Y() - this->truePose.Pos().Y(),2));
	  
	ROS_INFO_NAMED("ObjectChecker","Object ID: true = %s, submitted = %s result=%d; 2D position error = %.3f m",this->trueName.c_str(), _msg->header.frame_id.c_str(),(int)this->objectCorrect,this->objectError);
}

//////////////////////////////////////////////////
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


	/// \brief Link/model name for the object poses use as their frame of reference
  public: std::string frameName = std::string();
	  
    /// \brief Link/model that the object poses use as their frame of reference.
    public: physics::EntityPtr frame;

	/// \brief Current object that has been placed
    public: physics::EntityPtr curr_model;

    public: ignition::math::Pose3d orig_pose;
	  
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
    public: bool prefixObjectNames = false;

    /// \brief Id of first object to teleport.
    public: int startIndex = 0;

    /// \brief Last time (sim time) that the plugin was updated.
    public: gazebo::common::Time lastUpdateTime;

    /// \brief Counter for spawning objects with unique names on the belt.
    /// The key is the object type and the value contains the index of the next
    /// object to be spawned.
    public: std::map<std::string, int> objectCounter;

	/// \brief Implements ROS interface to recieve and check team submissions
    public: std::unique_ptr<ObjectChecker> objectChecker;

  };
}

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(PopulationPlugin)

/////////////////////////////////////////////////
PopulationPlugin::PopulationPlugin()
  : dataPtr(new PopulationPluginPrivate)
{
	gzmsg << "PopulationPlugin loaded" << std::endl;	
} 


/////////////////////////////////////////////////
PopulationPlugin::~PopulationPlugin()
{
}

/////////////////////////////////////////////////
void PopulationPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  
  // Base class, also binds the update method for the base class
  ScoringPlugin::Load(_world,_sdf);

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
    this->dataPtr->frameName = _sdf->Get<std::string>("frame");
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

	// Parse the object name - this is what must be matched id success.
    if (!objectElem->HasElement("name"))
    {
      gzerr << "PopulationPlugin: Unable to find <name> in object.\n";
      objectElem = objectElem->GetNextElement("object");
      continue;
    }
    sdf::ElementPtr nameElement = objectElem->GetElement("name");
    std::string name = nameElement->Get<std::string>();

    // Parse the object pose (optional).
    ignition::math::Pose3d pose;
    if (objectElem->HasElement("pose"))
    {
      sdf::ElementPtr poseElement = objectElem->GetElement("pose");
      pose = poseElement->Get<ignition::math::Pose3d>();
    }

    // Add the object to the collection.
    PopulationPluginPrivate::Object obj = {time, type, name, pose};
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

  this->dataPtr->lastUpdateTime = this->dataPtr->world->GetSimTime();

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

  // Optional: ROS namespace.
  std::string ns;
  if (_sdf->HasElement("robot_namespace"))
    ns = _sdf->GetElement("robot_namespace")->Get<std::string>();
  
  // Optional: ROS topic.
  std::string object_topic = "/vrx/perception/landmark";
  if (_sdf->HasElement("landmark_topic"))
  {
    object_topic =
      _sdf->GetElement("landmark_topic")->Get<std::string>();
  }
  // Instatiate the object checker
  this->dataPtr->objectChecker.reset(
	  new ObjectChecker(ns,object_topic,_world));
  
  
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
    this->dataPtr->world->GetSimTime() - this->dataPtr->startTime;

  gzmsg << "Object population paused" << std::endl;
}

/////////////////////////////////////////////////
void PopulationPlugin::Resume()
{
  if (this->dataPtr->enabled)
    return;

  this->dataPtr->enabled = true;
  this->dataPtr->startTime = this->dataPtr->world->GetSimTime() -
    this->dataPtr->elapsedWhenPaused;

   gzmsg << "Object population resumed" << std::endl;
}

/////////////////////////////////////////////////
void PopulationPlugin::Restart()
{
  this->dataPtr->enabled = true;
  this->dataPtr->elapsedEquivalentTime = 0;
  this->dataPtr->startTime = this->dataPtr->world->GetSimTime();
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
    this->dataPtr->lastUpdateTime = this->dataPtr->world->GetSimTime();
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
      this->dataPtr->lastUpdateTime = this->dataPtr->world->GetSimTime();
      return;
    }
  }

  // Check whether spawn a new object from the list.
  auto elapsedTime = this->dataPtr->world->GetSimTime() - this->dataPtr->lastUpdateTime;
  // The rate modifier has the effect of slowing down sim time in this plugin.
  // If the rate modifier is 0.5, we will wait for twice as much time before spawning a new object.
  this->dataPtr->elapsedEquivalentTime += elapsedTime.Double() * this->dataPtr->rateModifier;
  if (this->dataPtr->elapsedEquivalentTime >= this->dataPtr->objects.front().time)
  {
	gzmsg << "PopulationPlugin: spawn next object." << std::endl;
    auto obj = this->dataPtr->objects.front();

	// Try to use a model/link frame if specified.
	if (!this->dataPtr->frameName.empty())
	{
		this->dataPtr->frame = this->dataPtr->world->GetEntity(this->dataPtr->frameName);
		if (!this->dataPtr->frame)
		{
			gzthrow(std::string("The frame '") + this->dataPtr->frameName + "' does not exist");
		}
		if (!this->dataPtr->frame->HasType(physics::Base::LINK) &&
			!this->dataPtr->frame->HasType(physics::Base::MODEL))
		{
			gzthrow("'frame' tag must list the name of a link or model");
		}	
	}

    if (this->dataPtr->frame)
    {
		// Only use translation of frame pose
		// This is a bit of a hack to deal with transients of spawning buoys with significant non-zero attitude.
		//ignition::math::Pose3d framePose = this->dataPtr->frame->GetWorldPose().Ign();
		ignition::math::Pose3d framePose(this->dataPtr->frame->GetWorldPose().Ign().Pos(),ignition::math::Quaterniond());
		ignition::math::Matrix4d transMat(framePose);
		ignition::math::Matrix4d pose_local(obj.pose);
		obj.pose = (transMat * pose_local).Pose();
		
    }

    std::string modelName = obj.type;
    if (this->dataPtr->prefixObjectNames)
    {
      modelName = this->GetHandle() + "|" + modelName;
    }

	// Setup new trial in object checker
	this->dataPtr->objectChecker->NewTrial(obj.name,
										   obj.pose);
	
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

	// If necessary, move object back to original pose
	if (this->dataPtr->curr_model){
		this->dataPtr->curr_model->SetWorldPose(this->dataPtr->orig_pose);
	}
    // Get a unique name for the new object.
    modelName += "_" + std::to_string(index);
    auto modelPtr = this->dataPtr->world->GetEntity(modelName);
    if (modelPtr)
    {
		gzmsg << modelPtr->GetName() << std::endl;
		for (int ii = 0; ii < modelPtr->GetChildCount(); ii++){
			gzmsg << modelPtr->GetChild(ii)->GetName() << std::endl;
		}
		// Save for later
		this->dataPtr->curr_model = modelPtr;
		this->dataPtr->orig_pose = modelPtr->GetWorldPose().Ign();
		
		// Move it to the target pose.
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
    this->dataPtr->elapsedEquivalentTime = 0.0;
  }
    this->dataPtr->lastUpdateTime = this->dataPtr->world->GetSimTime();
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
  gazebo::common::Time curTime = this->dataPtr->world->GetSimTime();
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

//////////////////////////////////////////////////
void PopulationPlugin::OnRunning()
{
	this->dataPtr->objectChecker->Enable();
}
