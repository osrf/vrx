/*
 * Copyright (C) 2019  Rhys Mainwaring
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

#include "wave_gazebo_plugins/WavefieldModelPlugin.hh"
#include "wave_gazebo_plugins/Wavefield.hh"
#include "wave_gazebo_plugins/WavefieldEntity.hh"
#include "wave_gazebo_plugins/Utilities.hh"

#include <gazebo/common/Assert.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/msgs/any.pb.h>
#include <gazebo/msgs/empty.pb.h>
#include <gazebo/msgs/gz_string.pb.h>
#include <gazebo/msgs/param.pb.h>
#include <gazebo/msgs/param_v.pb.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <thread>

using namespace gazebo;

namespace asv
{

  GZ_REGISTER_MODEL_PLUGIN(WavefieldModelPlugin)

///////////////////////////////////////////////////////////////////////////////
// WavefieldModelPluginPrivate

  /// \internal
  /// \brief Private data for the WavefieldModelPlugin
  class WavefieldModelPluginPrivate
  {
    /// \brief World pointer.
    public: physics::WorldPtr world;

    /// \brief Model pointer.
    public: physics::ModelPtr model;

    /// \brief WavefieldEntity pointer.
    public: boost::shared_ptr<::asv::WavefieldEntity> wavefieldEntity;

    /// \brief Set the wavefield to be static [false].
    public: bool isStatic;

    /// \brief Update rate [30].
    public: double updateRate;

    /// \brief Previous update time.
    public: common::Time prevTime;

    /// \brief Connection to the World Update events.
    public: event::ConnectionPtr updateConnection;

    /// \brief Ignition transport node for igntopic "/marker".
    public: ignition::transport::Node ignNode;

    /// \brief Gazebo transport node.
    public: transport::NodePtr gzNode;

    /// \brief Publish to gztopic "~/reponse".
    public: transport::PublisherPtr responsePub;

    /// \brief Subscribe to gztopic "~/request".
    public: transport::SubscriberPtr requestSub;

    /// \brief Subscribe to gztopic "~/wave".
    public: transport::SubscriberPtr waveSub;
  };

///////////////////////////////////////////////////////////////////////////////
// WavefieldModelPlugin

  WavefieldModelPlugin::~WavefieldModelPlugin()
  {
    // Clean up.
    this->data->wavefieldEntity.reset();

    // Reset connections and transport.
    this->data->updateConnection.reset();
    this->data->requestSub.reset();
    this->data->waveSub.reset();
    this->data->responsePub.reset();
    this->data->gzNode->Fini();
    this->Fini();
  }

  WavefieldModelPlugin::WavefieldModelPlugin() : 
    ModelPlugin(), 
    data(new WavefieldModelPluginPrivate())
  {
  }

  void WavefieldModelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // @DEBUG_INFO
    // std::thread::id threadId = std::this_thread::get_id();
    // gzmsg << "Load WavefieldModelPlugin [thread: " << threadId << "]" << std::endl;

    GZ_ASSERT(_model != nullptr, "Invalid parameter _model");
    GZ_ASSERT(_sdf != nullptr, "Invalid parameter _sdf");

    // Capture the Model pointer
    this->data->model = _model;
    this->data->world = _model->GetWorld();
    GZ_ASSERT(this->data->world != nullptr, "Model has invalid World");

    // Transport
    this->data->gzNode = transport::NodePtr(new transport::Node());
    this->data->gzNode->Init(this->data->world->Name());

    // Publishers
    this->data->responsePub 
      = this->data->gzNode->Advertise<msgs::Response>("~/response");

    // Subscribers
    this->data->requestSub = this->data->gzNode->Subscribe(
      "~/request", &WavefieldModelPlugin::OnRequest, this);

    this->data->waveSub = this->data->gzNode->Subscribe(
      "~/wave", &WavefieldModelPlugin::OnWaveMsg, this);

    // Bind the update callback to the world update event 
    this->data->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&WavefieldModelPlugin::OnUpdate, this));

    // Parameters
    this->data->isStatic = Utilities::SdfParamBool(*_sdf, "static", false);
    this->data->updateRate = Utilities::SdfParamDouble(*_sdf, "update_rate", 30.0);

    // Wavefield
    this->data->wavefieldEntity.reset(new ::asv::WavefieldEntity(this->data->model));
    this->data->wavefieldEntity->Load(_sdf);
    this->data->wavefieldEntity->Init();

    // Generate the entity name and add as a child
    this->data->wavefieldEntity->SetName(
      WavefieldEntity::MakeName(this->data->model->GetName()));
    this->data->model->AddChild(this->data->wavefieldEntity);
  }

  void WavefieldModelPlugin::Init()
  {
    // @DEBUG_INFO
    // std::thread::id threadId = std::this_thread::get_id();
    // gzmsg << "Init WavefieldModelPlugin [thread: " << threadId << "]" << std::endl;
  }

  void WavefieldModelPlugin::Fini()
  {
  }

  void WavefieldModelPlugin::Reset()
  {
    // Reset time
    this->data->prevTime = this->data->world->SimTime(); 
  }

  void WavefieldModelPlugin::OnUpdate()
  {
    GZ_ASSERT(this->data->world != nullptr, "World is NULL");
    GZ_ASSERT(this->data->model != nullptr, "Model is NULL");
    GZ_ASSERT(this->data->wavefieldEntity != nullptr, "Wavefield Entity is NULL");

    if (!this->data->isStatic)
    {
      // Throttle update [30 FPS by default]
      auto updatePeriod = 1.0/this->data->updateRate;
      auto currentTime = this->data->world->SimTime();
      if ((currentTime - this->data->prevTime).Double() < updatePeriod)
      {
        return;
      }
      this->data->prevTime = currentTime; 

      // Wavefield
      this->data->wavefieldEntity->Update();
    }
  }

  std::shared_ptr<const WaveParameters> WavefieldModelPlugin::GetWaveParams(
    gazebo::physics::WorldPtr _world,
    const std::string& _waveModelName)
  {
    GZ_ASSERT(_world != nullptr, "World is null");

    physics::ModelPtr wavefieldModel = _world->ModelByName(_waveModelName);    
    if(wavefieldModel == nullptr)
    {
      gzerr << "No Wavefield Model found with name '" << _waveModelName << "'." << std::endl;
      return nullptr;
    }

    std::string wavefieldEntityName(WavefieldEntity::MakeName(_waveModelName));

    physics::BasePtr base = wavefieldModel->GetChild(wavefieldEntityName);
    boost::shared_ptr<WavefieldEntity> wavefieldEntity 
      = boost::dynamic_pointer_cast<WavefieldEntity>(base);
    if (wavefieldEntity == nullptr)
    {
      gzerr << "Wavefield Entity is null: " << wavefieldEntityName << std::endl;
      return nullptr;
    }
	// BSB - appears this method is not a part of the feature/vrx branch?
    //GZ_ASSERT(wavefieldEntity->GetWavefield() != nullptr, "Wavefield is null.");

    return wavefieldEntity->GetWaveParams();
  }

  // See for example: gazebo/physics/Wind.cc
  void WavefieldModelPlugin::OnRequest(ConstRequestPtr &_msg)
  {
    GZ_ASSERT(_msg != nullptr, "Request message must not be null");
    
    if (_msg->request() == "wave_param")
    {
      auto waveParams = this->data->wavefieldEntity->GetWaveParams();

      msgs::Param_V waveMsg;
      waveParams->FillMsg(waveMsg);

      msgs::Response response;
      response.set_id(_msg->id());
      response.set_request(_msg->request());
      response.set_response("success");
      std::string *serializedData = response.mutable_serialized_data();
      response.set_type(waveMsg.GetTypeName());
      waveMsg.SerializeToString(serializedData);
      this->data->responsePub->Publish(response);
    }
  }

  // @TODO_FRAGILE - the Entity needs proper clone and set methods to be safe
  void WavefieldModelPlugin::OnWaveMsg(ConstParam_VPtr &_msg)
  {
    GZ_ASSERT(_msg != nullptr, "Wave message must not be null");

    // Update wave params 
    auto constWaveParams = this->data->wavefieldEntity->GetWaveParams();
    GZ_ASSERT(constWaveParams != nullptr, "WaveParameters must not be null");
    auto& waveParams = const_cast<WaveParameters&>(*constWaveParams);
    waveParams.SetFromMsg(*_msg);

    // @DEBUG_INFO
    gzmsg << "Wavefield Model received message on topic ["
      << this->data->waveSub->GetTopic() << "]" << std::endl;
    waveParams.DebugPrint();
  }
} // namespace gazebo
