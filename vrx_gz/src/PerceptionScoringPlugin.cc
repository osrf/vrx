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

#include <gz/msgs/pose.pb.h>
#include <memory>
#include <string>
#include <vector>
#include <gz/common/Profiler.hh>
#include <gz/math/Matrix4.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
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

  /// \brief Amount of time in which the object should be spawned.
  public: double duration = 5;

  /// \brief Object type.
  public: std::string type;

  /// \brief Object name.
  public: std::string name;

  /// \brief Pose in which the object should be placed relative to the
  /// specified frame.
  public: math::Pose3d trialPose;

  /// \brief Pose in which the object should be placed when it's no active
  /// in global frame.
  public: math::Pose3d origPose;

  /// \brief The model entity.
  public: sim::Entity entity = sim::kNullEntity;

  /// \brief Model entity that this object is representing.
  public: std::unique_ptr<sim::Model> model;

  /// \brief bool to tell weather or not the object is open for attempts.
  public: bool active = false;

  /// \brief Error associated with the guess of a model.
  public: double error = 10.0;

  /// \brief The Entity Component Manager.
  public: sim::EntityComponentManager &ecm;

  /// \brief constructor of perception object.
  /// \param[in] _time Simulation time in which the object should be spawned.
  /// \param[in] _duration Amount of time in which the object should be spawned.
  /// \param[in] _type Object type.
  /// \param[in] _name Object name.
  /// \param[in] _trialPose Pose in which the object should be placed relative
  ///                       to the specified frame.
  /// \param[in out] _ecm The Entity Component Manager.
  public: PerceptionObject(const double _time,
                           const double _duration,
                           const std::string &_type,
                           const std::string &_name,
                           const math::Pose3d &_trialPose,
                           sim::EntityComponentManager &_ecm);

  /// \brief Set the error of this object if this object is active and this is
  /// the lowest seen error.
  /// \param[in] _error New error.
  public: void SetError(const double _error);

  /// \brief Move the object to where it is supposed to be relative to the frame
  /// of the robot and make it active
  /// \param[in] _frame The entity to be used as reference frame.
  public: void StartTrial(const sim::Entity _frame);

  /// \brief Move the object back to its original location and make inactive.
  public: void EndTrial();

  /// \brief Stream extraction operator.
  /// \param[in] _out output stream.
  /// \param[in] _obj PerceptionObject to output.
  /// \return The stream.
  public: friend std::ostream &operator<<(std::ostream &_out,
                                          const PerceptionObject &_obj)
  {
    _out << "\tName: "       << _obj.name << std::endl
         << "\tType: "       << _obj.type << std::endl
         << "\tTime: "       << _obj.time << std::endl
         << "\tDuration: "   << _obj.duration << std::endl
         << "\tOrig pose: "  << _obj.origPose << std::endl
         << "\tTrial pose: " << _obj.trialPose << std::endl
         << "\tEntity: "     << _obj.entity << std::endl
         << "\tActive: "     << std::boolalpha << _obj.active
                             << std::noboolalpha << std::endl
         << "\tError: "      << _obj.error << std::endl;

    return _out;
  }
};

/////////////////////////////////////////////////
PerceptionObject::PerceptionObject(const double _time, const double _duration,
  const std::string &_type, const std::string &_name,
  const math::Pose3d &_trialPose, sim::EntityComponentManager &_ecm)
  : time(_time),
    duration(_duration),
    type(_type),
    name(_name),
    trialPose(_trialPose),
    ecm(_ecm)
{
  auto entities = sim::entitiesFromScopedName(_name, _ecm);
  if (entities.empty())
  {
    gzerr << "Unable to find entity [" << _name << "]" << std::endl;
    return;
  }

  this->entity = *entities.begin();
  this->model.reset(new sim::Model(this->entity));
  this->origPose = _ecm.Component<sim::components::Pose>(this->entity)->Data();
}

/////////////////////////////////////////////////
void PerceptionObject::SetError(const double _error)
{
  if (this->active)
    this->error = std::min(2.0, std::min(this->error, _error));
}

/////////////////////////////////////////////////
void PerceptionObject::StartTrial(const sim::Entity _frame)
{
  // Set object pose relative to the specified frame (e.g., the wam-v).
  auto frameWorldPose =
    this->ecm.Component<sim::components::Pose>(_frame)->Data();

  math::Pose3d framePose(
    frameWorldPose.Pos(),
    math::Quaterniond(0.0, 0.0, frameWorldPose.Rot().Yaw()));
  math::Matrix4d transMat(framePose);
  math::Matrix4d poseLocal(this->trialPose);

  this->model->SetWorldPoseCmd(this->ecm, (transMat * poseLocal).Pose());
  this->active = true;

  gzdbg << "PerceptionScoringPlugin: spawning " << this->name << std::endl;
}

/////////////////////////////////////////////////
void PerceptionObject::EndTrial()
{
  this->model->SetWorldPoseCmd(this->ecm, this->origPose);
  this->active = false;

  gzdbg << "PerceptionScoringPlugin: despawning " << this->name << std::endl;
}

/// \brief Private PerceptionScoringPlugin data class.
class PerceptionScoringPlugin::Implementation
{
  /// \brief Parse all SDF parameters.
  /// \param[in] _ecm The Entity Component Manager.
  /// \return True when all params were successfully parsed or false otherwise.
  public: bool ParseSDFParameters(sim::EntityComponentManager &_ecm);

  /// \brief Register a new perception attempt request.
  /// \param[in] _msg The message containing a perception attempt.
  public: void OnAttempt(const msgs::Pose &_msg);

  /// \brief Process all pending perception requests.
  /// \param[in] _ecm The Entity Component Manager.
  public: void ProcessAttempts(sim::EntityComponentManager &_ecm);

  /// \brief Number of perception attempts (equal to the total number of objects
  /// spawned.
  public: uint16_t attemptBal = 0u;

  /// \brief Topic where the object id/pose is received.
  public: std::string objectTopic;

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief SDF pointer.
  public: sdf::ElementPtr sdf;

  /// \brief Collection of objects to be spawned.
  public: std::vector<PerceptionObject> objects;

  /// \brief Name of the object used as the frame of reference.
  public: std::string frameName = "";

  /// \brief Entity of the object used as the frame of reference.
  public: sim::Entity frame = sim::kNullEntity;

  /// \brief count of how many objects have been despawned
  public: uint16_t objectsDespawned = 0u;

  /// \brief World pointer.
  public: std::unique_ptr<sim::World> world;

  /// \brief Used to parse the SDF during the first plugin iteration.
  public: bool firstIteration = true;

  /// \brief Current vector of perception requests to be processed.
  public: std::vector<msgs::Pose> requests;

  /// \brief Mutex to protect the requests.
  public: std::mutex mutex;
};

//////////////////////////////////////////////////
bool PerceptionScoringPlugin::Implementation::ParseSDFParameters(
  sim::EntityComponentManager &_ecm)
{
  auto worldEntity = _ecm.EntityByComponents(sim::components::World());
  this->world.reset(new sim::World(worldEntity));

  if (!this->sdf->HasElement("object_sequence"))
  {
    gzerr << "PerceptionScoringPlugin: Unable to find <object_sequence> element"
          << std::endl;
    return false;
  }

  sdf::ElementPtr sequence = this->sdf->GetElement("object_sequence");

  sdf::ElementPtr objectElem = nullptr;
  if (sequence->HasElement("object"))
    objectElem = sequence->GetElement("object");

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
    PerceptionObject obj(time, duration, type, name, pose, _ecm);
    gzdbg << "New object: " << std::endl << obj << std::endl;
    this->objects.push_back(std::move(obj));

    objectElem = objectElem->GetNextElement("object");
  }

  // Optional: <frame>.
  if (this->sdf->HasElement("frame"))
    this->frameName = this->sdf->Get<std::string>("frame");

  // Optional: <landmark_topic>.
  this->objectTopic = "/vrx/perception/landmark";
  if (this->sdf->HasElement("landmark_topic"))
  {
    this->objectTopic =
      this->sdf->GetElement("landmark_topic")->Get<std::string>();
  }

  return true;
}

//////////////////////////////////////////////////
void PerceptionScoringPlugin::Implementation::OnAttempt(const msgs::Pose &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->requests.push_back(_msg);
}

//////////////////////////////////////////////////
void PerceptionScoringPlugin::Implementation::ProcessAttempts(
  sim::EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  for (auto &_msg : this->requests)
  {
    // Only accept an attempt if there are any in the attempt balance.
    if (this->attemptBal == 0)
    {
      gzwarn << "PerceptionScoring: Attempt Balance is 0, no attempts currently"
             << " allowed. Ignoring." << std::endl;

      this->requests.clear();
      return;
    }

    // In the ROS side, the object type is reported in the "frame_id" field
    // of the header.
    std::string typeReported;
    for (auto i = 0u; i < _msg.header().data_size(); ++i)
    {
      if (_msg.header().data(i).key() == "frame_id")
      {
        typeReported = _msg.header().data(i).value(0);
        break;
      }
    }

    // Burn one attempt.
    --this->attemptBal;
    gzdbg << "PerceptionScoring: New Attempt Balance: " << this->attemptBal
          << std::endl;

    for (auto &obj : this->objects)
    {
      // If attempt correct type.
      if (obj.type == typeReported)
      {
        // Convert geo pose to Gazebo pose.
        math::Vector3d scVec(_msg.position().x(), _msg.position().y(), 0);
        math::Vector3d cartVec = this->world->SphericalCoordinates(
          _ecm)->LocalFromSphericalPosition(scVec);

        // Get current pose of the current object.
        math::Pose3d truePose =
          _ecm.Component<sim::components::Pose>(obj.entity)->Data();

        // 2D Error.
        double error = sqrt(pow(cartVec.X() - truePose.Pos().X(), 2) +
                            pow(cartVec.Y() - truePose.Pos().Y(), 2));
        obj.SetError(error);
      }
    }
  }
  this->requests.clear();
}

//////////////////////////////////////////////////
PerceptionScoringPlugin::PerceptionScoringPlugin()
  : ScoringPlugin(),
    dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void PerceptionScoringPlugin::Configure(const sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  sim::EntityComponentManager &_ecm, sim::EventManager &_eventMgr)
{
  ScoringPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  this->dataPtr->sdf = _sdf->Clone();
}

//////////////////////////////////////////////////
void PerceptionScoringPlugin::PreUpdate(const sim::UpdateInfo &_info,
  sim::EntityComponentManager &_ecm)
{
  // don't update when paused
  if (_info.paused)
    return;

  ScoringPlugin::PreUpdate(_info, _ecm);

  if (this->TaskState() == "finished")
    return;

  // SDF.
  if (this->dataPtr->firstIteration)
  {
    this->dataPtr->firstIteration = false;
    if (!this->dataPtr->ParseSDFParameters(_ecm))
    {
      gzerr << "Perception scoring disabled" << std::endl;
      return;
    }
  }

  // Read the frame if we haven't done it yet.
  if (this->dataPtr->frame == sim::kNullEntity &&
      !this->dataPtr->frameName.empty())
  {
    // Get reference frame.
    auto entities = sim::entitiesFromScopedName(this->dataPtr->frameName, _ecm);
    if (entities.empty())
      return;

    this->dataPtr->frame = *entities.begin();
  }

  // Check all objects.
  for (auto &obj : this->dataPtr->objects)
  {
    // Time to spawn an object.
    if (this->ElapsedTime().count() > obj.time &&
        this->ElapsedTime().count() < obj.time + obj.duration && !obj.active)
    {
      // Increment the atempt balance for this new obj
      ++this->dataPtr->attemptBal;
      obj.StartTrial(this->dataPtr->frame);
      gzdbg << "PerceptionScoring: New Attempt Balance: "
            << this->dataPtr->attemptBal << std::endl;
    }

    // Time to despawn and object
    if (this->ElapsedTime().count() > obj.time + obj.duration && obj.active)
    {
      // Prevent negative attemp balance.
      if (this->dataPtr->attemptBal > 0)
        --this->dataPtr->attemptBal;
      // Increment objects despawned.
      ++this->dataPtr->objectsDespawned;
      obj.EndTrial();

      // Add the score for this object.
      this->SetScore(this->Score() + obj.error);

      gzdbg << "PerceptionScoring: New Attempt Balance: "
            << this->dataPtr->attemptBal << std::endl;
    }
  }

  // If we have finished.
  if (this->dataPtr->objectsDespawned == this->dataPtr->objects.size() &&
      this->TaskState() != "finished")
  {
    // Publish string summarizing the objects
    for (auto &obj : this->dataPtr->objects)
      gzdbg << "PerceptionScoring: " << std::endl << obj << std::endl;

    // Run score is the mean error per object.
    this->SetScore(this->Score() / this->dataPtr->objects.size());
    gzdbg << "Perception run score: " << this->Score() << std::endl;

    this->Finish();
  }

  // If we have requests, let's process them.
  this->dataPtr->ProcessAttempts(_ecm);
}

//////////////////////////////////////////////////
void PerceptionScoringPlugin::OnRunning()
{
  this->dataPtr->node.Subscribe(this->dataPtr->objectTopic,
    &PerceptionScoringPlugin::Implementation::OnAttempt, this->dataPtr.get());
  ScoringPlugin::OnRunning();
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
