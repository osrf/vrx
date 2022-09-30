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

#include <memory>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Matrix4.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

#include "PerceptionScoringPlugin.hh"

using namespace gz;
using namespace vrx;

/// \brief Class to store information about each object to be populated.
class PerceptionObject
{
  /// \brief Simulation time in which the object should be spawned.
  public: double time;

  /// \brief amount of time in which the object should be spawned.
  public: double duration;

  /// \brief Object type.
  public: std::string type;

  /// \brief Object name.
  public: std::string name;

  /// \brief Pose in which the object should be placed relative to the
  /// specified frame.
  public: math::Pose3d trialPose;

  /// \brief Pose in which the object should be placed in global frame.
  public: math::Pose3d origPose;

  /// \brief Model entity that this object is representing.
  public: std::unique_ptr<sim::Model> model;

  /// \brief bool to tell weather or not the object is open for attempts.
  public: bool active = false;

  /// \brief Error associated with the guess of a model.
  public: double error = -1.0;

  /// \brief Pointer to the ECM.
  public: sim::EntityComponentManager *ecm;

  /// \brief constructor of perception object.
  /// \param ToDo.
  public: PerceptionObject(const double _time,
                           const double _duration,
                           const std::string &_type,
                           const std::string &_name,
                           const math::Pose3d &_trialPose,
                           const sim::Entity _entity);

  /// \brief set the error of this boject if this object is active
  /// and this is the lowest seen error.
  /// \param[in] _error TODO.
  public: void SetError(const double _error);

  /// \brief move the object to where it is supposed to be relative to the frame
  /// \brief of the robot and make it active
  /// \param[in] _frame TODO.
  public: void StartTrial(const sim::Link _frame);

  /// \brief move the object back to its original location and make inactive.
  public: void EndTrial();

  /// \return a string summarizing this object.
  public: std::string Str();
};

/////////////////////////////////////////////////
PerceptionObject::PerceptionObject(const double _time, const double _duration,
  const std::string &_type, const std::string &_name,
  const math::Pose3d &_trialPose, const sim::Entity _world)
{
  this->time = _time;
  this->duration = _duration;
  this->type = _type;
  this->name = _name;
  this->trialPose = _trialPose;
  // #if GAZEBO_MAJOR_VERSION >= 8
  //   this->modelPtr = _world->EntityByName(this->name);
  // #else
  //   this->modelPtr = _world->GetEntity(this->name);
  // #endif
  // if (modelPtr)
  // {
  //   #if GAZEBO_MAJOR_VERSION >= 8
  //     this->origPose = this->modelPtr->WorldPose();
  //   #else
  //     this->origPose = this->modelPtr->GetWorldPose().Ign();
  //   #endif
  // }
}

/////////////////////////////////////////////////
std::string PerceptionObject::Str()
{
  std::string rtn = "\nname: ";
  rtn += this->name;
  rtn += "\ntype: ";
  rtn += this->type;
  rtn += "\ntime: ";
  rtn += std::to_string(this->time);
  rtn +=  "\nduration: ";
  rtn += std::to_string(this->duration);
  rtn +=  "\nerror: ";
  rtn += std::to_string(this->error);
  return rtn;
}

/////////////////////////////////////////////////
void PerceptionObject::SetError(const double _error)
{
  if (this->active)
    this->error = std::min(2.0, std::min(this->error, _error));
}

/////////////////////////////////////////////////
void PerceptionObject::StartTrial(const sim::Link _frame)
{
  // Set object pose relative to the specified frame (e.g., the wam-v).
  // Pitch and roll are set to zero as a hack to deal with
  // transients associated with spawning buoys with significant attitude.
  math::Pose3d framePose(
    _frame.WorldPose(*this->ecm)->Pos(),
    math::Quaterniond(0.0, 0.0, _frame.WorldPose(*this->ecm)->Rot().Yaw()));
  math::Matrix4d transMat(framePose);
  math::Matrix4d pose_local(this->trialPose);

  // this->model->SetWorldPose((transMat * pose_local).Pose());
  this->model->SetWorldPoseCmd(*ecm, (transMat * pose_local).Pose());
  // this->modelPtr->SetWorldTwist(ign_math_vector3d_zero, ign_math_vector3d_zero);

  this->active = true;

  gzmsg << "PerceptionScoringPlugin: spawning " << this->name << std::endl;
}

/////////////////////////////////////////////////
void PerceptionObject::EndTrial()
{
  this->model->SetWorldPoseCmd(*ecm, this->origPose);
  // this->modelPtr->SetWorldTwist(ign_math_vector3d_zero,
  //   ign_math_vector3d_zero);
  this->active = false;

  gzmsg << "PerceptionScoringPlugin: despawning " << this->name << std::endl;
}

/// \brief Private PerceptionScoringPlugin data class.
class PerceptionScoringPlugin::Implementation
{
  /// \brief Parse all SDF parameters.
  /// \return True when all parameters were successfully parsed or false
  /// otherwise.
  public: bool ParseSDFParameters();

  /// \brief Restart the object population list.
  public: void Restart();

  /// \brief
  /// \param[in] _msg The message containing a perception attempt.
  public: void OnAttempt(const msgs::Pose &_msg);

  /// \brief TODO.
  public: int attemptBal = 0;

  /// \brief ROS namespace.
  public: std::string ns;

  /// \brief ROS topic where the object id/pose is received.
  public: std::string objectTopic;

  /// \brief ROS Node handle.
  //public: ros::NodeHandle nh;

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief ROS subscriber
  // public: ros::Subscriber objectSub;

  /// \brief World pointer.
  // public: gazebo::physics::WorldPtr world;

  /// \brief SDF pointer.
  public: sdf::ElementPtr sdf;

  /// \brief Collection of objects to be spawned.
  public: std::vector<PerceptionObject> objects;

  /// \brief Connection event.
  // public: gazebo::event::ConnectionPtr connection;

  /// \brief The time specified in the object is relative to this time.
  public: double startTime;

  /// \brief When true, "objects" will be repopulated when the object queue
  /// is empty, creating an infinite supply of objects.
  public: bool loopForever = false;

  /// \brief Link/model name for the object poses use as their frame of
  // reference.
  public: std::string frameName = std::string();

  /// \brief Link/model that the object poses use as their frame of reference.
  public: std::unique_ptr<sim::Link> frame;

  /// \brief Last time (sim time) that the plugin was updated.
  public: double lastUpdateTime;

  /// \ brief count of how many objects have been despawned
  public: int objectsDespawned = 0;
};

//////////////////////////////////////////////////
bool PerceptionScoringPlugin::Implementation::ParseSDFParameters()
{
  if (this->sdf->HasElement("loop_forever"))
  {
    sdf::ElementPtr loopElem = this->sdf->GetElement("loop_forever");
    this->loopForever = loopElem->Get<bool>();
  }

  if (this->sdf->HasElement("frame"))
  {
    this->frameName = this->sdf->Get<std::string>("frame");
  }

  if (!this->sdf->HasElement("object_sequence"))
  {
    gzerr << "PerceptionScoringPlugin: Unable to find <object_sequence> "
      "element\n";
    return false;
  }

  sdf::ElementPtr sequence = this->sdf->GetElement("object_sequence");

  sdf::ElementPtr objectElem = nullptr;
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

    double duration = 5;
    if (objectElem->HasElement("duration"))
    {
      sdf::ElementPtr durationElement = objectElem->GetElement("duration");
      duration = durationElement->Get<double>();
    }

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
    math::Pose3d pose = poseElement->Get<math::Pose3d>();

    // Add the object to the collection.
    // PerceptionObject obj(time, duration, type, name, pose, _world);

    // TODO: caguero
    //PerceptionObject obj(time, duration, type, name, pose, 0);
    //this->objects.push_back(obj);

    objectElem = objectElem->GetNextElement("object");
  }

  // this->lastUpdateTime = this->world->SimTime();

  // Optional: ROS namespace.
  // if (_sdf->HasElement("robot_namespace"))
  //   this->ns = _sdf->GetElement("robot_namespace")->Get<std::string>();

  // Optional: ROS topic.
  this->objectTopic = "/vrx/perception/landmark";
  if (this->sdf->HasElement("landmark_topic"))
  {
    this->objectTopic =
      this->sdf->GetElement("landmark_topic")->Get<std::string>();
  }

  return true;
}

/////////////////////////////////////////////////
void PerceptionScoringPlugin::Implementation::Restart()
{
  for (auto &obj : this->objects)
  {
    // reset all objects' errors
    obj.error = 10.0;
    // Bump all objs time to start again.
    // obj.time += this->world->SimTime().Double();
  }

  gzmsg << "Object population restarted" << std::endl;
}

//////////////////////////////////////////////////
PerceptionScoringPlugin::PerceptionScoringPlugin()
  : ScoringPlugin(),
    dataPtr(utils::MakeUniqueImpl<Implementation>())
{
  gzmsg << "Perception scoring plugin loaded" << std::endl;
}

//////////////////////////////////////////////////
void PerceptionScoringPlugin::Configure(const sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  sim::EntityComponentManager &_ecm, sim::EventManager &_eventMgr)
{
  ScoringPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);

  this->dataPtr->sdf = _sdf->Clone();

  // SDF.
  if (!this->dataPtr->ParseSDFParameters())
  {
    gzerr << "Perception scoring disabled" << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
void PerceptionScoringPlugin::PreUpdate(const sim::UpdateInfo &_info,
  sim::EntityComponentManager &_ecm)
{

}

//////////////////////////////////////////////////
void PerceptionScoringPlugin::OnRunning()
{
  gzmsg << "PerceptionScoringPlugin::OnRunning" << std::endl;

  // Quit if ros plugin was not loaded
  // this->objectSub = this->nh.subscribe(this->objectTopic, 1,
  //   &PerceptionScoringPlugin::OnAttempt, this);
}

//////////////////////////////////////////////////
void PerceptionScoringPlugin::ReleaseVehicle()
{
  // Avoid releasing the vehicle by overriding this function.
}

GZ_ADD_PLUGIN(PerceptionScoringPlugin,
              sim::System,
              PerceptionScoringPlugin::ISystemConfigure,
              PerceptionScoringPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::PerceptionScoringPlugin,
                    "vrx::PerceptionScoringPlugin")
