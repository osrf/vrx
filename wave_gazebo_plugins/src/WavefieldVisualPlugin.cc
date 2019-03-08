// Copyright (C) 2019  Rhys Mainwaring
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "asv_wave_sim_gazebo_plugins/WavefieldVisualPlugin.hh"
#include "asv_wave_sim_gazebo_plugins/Gazebo.hh"
#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"
#include "asv_wave_sim_gazebo_plugins/Utilities.hh"

#include <gazebo/gazebo.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include "gazebo/rendering/ogre_gazebo.h"
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <memory>
#include <thread>
#include <vector>

using namespace gazebo;

namespace asv
{

  GZ_REGISTER_VISUAL_PLUGIN(WavefieldVisualPlugin)

///////////////////////////////////////////////////////////////////////////////
// Utilties

  /// \brief Convert a vector containing two doubles to an Ogre Vector2.
  ///
  /// \param[in] _v       A std::vector containing two entries.
  /// \param[out] _vout   The Ogre vector to be populated.
  void ToOgreVector2(const std::vector<double>& _v, Ogre::Vector2& _vout)
  {
    _vout = Ogre::Vector2::ZERO;
    if (_v.size() > 2)
    {
      gzerr << "Vector must have size 2 or less" << std::endl;
      return;
    }
    for (size_t i=0; i<_v.size(); ++i)
    {
      _vout[i] = _v[i];
    }
  }

  /// \brief Convert a vector containing three doubles to an Ogre Vector3.
  ///
  /// \param[in] _v       A std::vector containing three entries.
  /// \param[out] _vout   The Ogre vector to be populated.
  void ToOgreVector3(const std::vector<double>& _v, Ogre::Vector3& _vout)
  {
    _vout = Ogre::Vector3::ZERO;
    if (_v.size() > 3)
    {
      gzerr << "Vector must have size 3 or less" << std::endl;
      return;
    }
    for (size_t i=0; i<_v.size(); ++i)
    {
      _vout[i] = _v[i];
    }
  }

  /// \brief Convert an ignition Vector2 to an Ogre Vector2.
  ///
  /// \param[in] _v       An ignition vector.
  /// \param[out] _vout   The Ogre vector to be populated.
  void ToOgreVector2(const ignition::math::Vector2d& _v, Ogre::Vector2& _vout)
  {
    _vout.x = _v.X();
    _vout.y = _v.Y();
  }

  /// \brief Convert an ignition Vector3 to an Ogre Vector3.
  ///
  /// \param[in] _v       An ignition vector.
  /// \param[out] _vout   The Ogre vector to be populated.
  void ToOgreVector3(const ignition::math::Vector3d& _v, Ogre::Vector3& _vout)
  {
    _vout.x = _v.X();
    _vout.y = _v.Y();
    _vout.z = _v.Z();
  }

  void ToOgreVector2(
    const std::vector<ignition::math::Vector2d>& _v,
    Ogre::Vector2& _vout0,
    Ogre::Vector2& _vout1,
    Ogre::Vector2& _vout2
  )
  {
    _vout0 = Ogre::Vector2::ZERO;
    _vout1 = Ogre::Vector2::ZERO;
    _vout2 = Ogre::Vector2::ZERO;

    if (_v.size() > 3)
    {
      gzerr << "Vector must have size 3 or less" << std::endl;
      return;
    }
    if (_v.size() > 0)
      ToOgreVector2(_v[0], _vout0);
    if (_v.size() > 1)
      ToOgreVector2(_v[1], _vout1);
    if (_v.size() > 2)
      ToOgreVector2(_v[2], _vout2);
  }
  
  void ToOgreVector3(
    const std::vector<ignition::math::Vector3d>& _v,
    Ogre::Vector3& _vout0,
    Ogre::Vector3& _vout1,
    Ogre::Vector3& _vout2
  )
  {
    _vout0 = Ogre::Vector3::ZERO;
    _vout1 = Ogre::Vector3::ZERO;
    _vout2 = Ogre::Vector3::ZERO;

    if (_v.size() > 3)
    {
      gzerr << "Vector must have size 3 or less" << std::endl;
      return;
    }
    if (_v.size() > 0)
      ToOgreVector3(_v[0], _vout0);
    if (_v.size() > 1)
      ToOgreVector3(_v[1], _vout1);
    if (_v.size() > 2)
      ToOgreVector3(_v[2], _vout2);
  }

///////////////////////////////////////////////////////////////////////////////
// WavefieldVisualPluginPrivate

  /// \internal
  /// \brief Private data for the WavefieldVisualPlugin
  class WavefieldVisualPluginPrivate
  {
    /// \brief The visual containing this plugin.
    public: rendering::VisualPtr visual;

    /// \brief The wavefield visual plugin SDF.
    public: sdf::ElementPtr sdf;

    /// \brief The wavefield parameters.
    public: std::shared_ptr<WaveParameters> waveParams;

    /// \brief Do not update visual if 'true', [false].
    public: bool isStatic;

    /// \brief World stats.
    public: double simTime, realTime, pauseTime;
    public: bool paused;

    /// \brief Prevent multiple calls to Init loading visuals twice...
    public: bool isInitialised;

    /// \brief Mutex
    public: std::recursive_mutex mutex;

    /// \brief Event based connections.
    public: event::ConnectionPtr connection;

    /// \brief Node used to establish communication with gzserver.
    public: transport::NodePtr gzNode;

    /// \brief Publish to gztopic "~/request".
    public: transport::PublisherPtr requestPub;

    /// \brief Subscribe to gztopic "~/response".
    public: transport::SubscriberPtr responseSub;

    /// \brief Subscribe to gztopic "~/wave".
    public: transport::SubscriberPtr waveSub;

    /// \brief Subscribe to gztopic "~/world_stats".
    public: transport::SubscriberPtr statsSub;
  };

///////////////////////////////////////////////////////////////////////////////
// WavefieldVisualPlugin

  WavefieldVisualPlugin::~WavefieldVisualPlugin()
  { 
    // Clean up.
    this->data->waveParams.reset();

    // Reset connections and transport.
    this->data->connection.reset();
    this->data->statsSub.reset();
    this->data->waveSub.reset();
    this->data->responseSub.reset();
    this->data->requestPub.reset();
    this->data->gzNode.reset();
  }

  WavefieldVisualPlugin::WavefieldVisualPlugin() :
    VisualPlugin(),
    data(new WavefieldVisualPluginPrivate)
  {
    this->data->isInitialised = false;
  }

  void WavefieldVisualPlugin::Load(
    rendering::VisualPtr _visual,
    sdf::ElementPtr _sdf)
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    // @DEBUG_INFO
    // std::thread::id threadId = std::this_thread::get_id();
    // gzmsg << "Load WavefieldVisualPlugin [thread: " << threadId << "]" << std::endl;

    // Capture visual and plugin SDF
    GZ_ASSERT(_visual != nullptr, "Visual must not be null");
    GZ_ASSERT(_sdf != nullptr, "SDF Element must not be null");

    // Capture the visual and sdf.
    this->data->visual = _visual;
    this->data->sdf = _sdf;

    // Transport
    this->data->gzNode = transport::NodePtr(new transport::Node());
    this->data->gzNode->Init();

    // Publishers
    this->data->requestPub 
      = this->data->gzNode->Advertise<msgs::Request>("~/request");

    // Subscribers
    this->data->responseSub = this->data->gzNode->Subscribe(
      "~/response", &WavefieldVisualPlugin::OnResponse, this);

    this->data->waveSub = this->data->gzNode->Subscribe(
      "~/wave", &WavefieldVisualPlugin::OnWaveMsg, this);

    this->data->statsSub = this->data->gzNode->Subscribe(
      "~/world_stats", &WavefieldVisualPlugin::OnStatsMsg, this);

    // Bind the update method to ConnectPreRender events
    this->data->connection = event::Events::ConnectPreRender(
        std::bind(&WavefieldVisualPlugin::OnUpdate, this));

    // Wave Parameters
    this->data->waveParams.reset(new WaveParameters());
    if (_sdf->HasElement("wave"))
    {
      sdf::ElementPtr sdfWave = _sdf->GetElement("wave");
      this->data->waveParams->SetFromSDF(*sdfWave);
    }

    // Plugin
    this->data->isStatic = Utilities::SdfParamBool(*_sdf, "static", false);

    // @DEBUG_INFO
    // gzmsg << "WavefieldVisualPlugin..." <<  std::endl;
    // this->data->waveParams->DebugPrint();
  }

  void WavefieldVisualPlugin::Init()
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    // @DEBUG_INFO
    // std::thread::id threadId = std::this_thread::get_id();
    // gzmsg << "Init WavefieldVisualPlugin [thread: " << threadId << "]" << std::endl;
  
    if (!this->data->isInitialised)
    {
      // Request "wave_param"
      msgs::RequestPtr requestMsg(msgs::CreateRequest("wave_param", ""));
      this->data->requestPub->Publish(*requestMsg);

      // Initialise vertex shader
      std::string shaderType = "vertex";
#if 0
      this->data->visual->SetMaterialShaderParam(
        "time", shaderType, std::to_string(0.0));
#else
      rendering::SetMaterialShaderParam(*this->data->visual,
        "time", shaderType, std::to_string(0.0));
#endif
      this->SetShaderParams();
      this->data->isInitialised = true;   
    }
  }

  void WavefieldVisualPlugin::Reset()
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    // @DEBUG_INFO
    // gzmsg << "Reset WavefieldVisualPlugin" << std::endl;
  }

  void WavefieldVisualPlugin::OnUpdate()
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    if (!this->data->isStatic && !this->data->paused)
    { 
      std::string shaderType = "vertex";
      float simTime = this->data->simTime;
#if 0
      this->data->visual->SetMaterialShaderParam(
        "time", shaderType, std::to_string(simTime));
#else
      rendering::SetMaterialShaderParam(*this->data->visual,
        "time", shaderType, std::to_string(simTime));
#endif
    }
  }

  void WavefieldVisualPlugin::OnResponse(ConstResponsePtr &_msg)
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    GZ_ASSERT(_msg != nullptr, "Response message must not be null");

    msgs::Param_V windMsg;
    if (_msg->type() == windMsg.GetTypeName())
    {
      // Parse the response
      windMsg.ParseFromString(_msg->serialized_data());

      // Update wave params and vertex shader 
      this->data->waveParams->SetFromMsg(windMsg);
      this->SetShaderParams();

      // @DEBUG_INFO
      gzmsg << "Wavefield Visual received message on topic [" 
        << this->data->responseSub->GetTopic() << "]" << std::endl;
      this->data->waveParams->DebugPrint();
    }
  }

  void WavefieldVisualPlugin::OnWaveMsg(ConstParam_VPtr &_msg)
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    GZ_ASSERT(_msg != nullptr, "Wave message must not be null");

    // Update wave params and vertex shader 
    this->data->waveParams->SetFromMsg(*_msg);
    this->SetShaderParams();

    // @DEBUG_INFO
    gzmsg << "Wavefield Visual received message on topic [" 
      << this->data->waveSub->GetTopic() << "]" << std::endl;
    this->data->waveParams->DebugPrint();
  }

  void WavefieldVisualPlugin::OnStatsMsg(ConstWorldStatisticsPtr &_msg)
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    this->data->simTime = gazebo::msgs::Convert(_msg->sim_time()).Double();
    this->data->realTime = gazebo::msgs::Convert(_msg->real_time()).Double();
    this->data->pauseTime = gazebo::msgs::Convert(_msg->pause_time()).Double();
    this->data->paused = _msg->paused();
  }

  void WavefieldVisualPlugin::SetShaderParams()
  {
    std::lock_guard<std::recursive_mutex> lock(this->data->mutex);

    const std::string shaderType = "vertex";

    // Parameters (to load from SDF)
    Ogre::Vector3 amplitude = Ogre::Vector3::ZERO;
    Ogre::Vector3 wavenumber = Ogre::Vector3::ZERO;
    Ogre::Vector3 omega = Ogre::Vector3::ZERO;
    Ogre::Vector3 steepness = Ogre::Vector3::ZERO;
    Ogre::Vector2 dir0 = Ogre::Vector2::ZERO;
    Ogre::Vector2 dir1 = Ogre::Vector2::ZERO;
    Ogre::Vector2 dir2 = Ogre::Vector2::ZERO;
    
    ToOgreVector3(this->data->waveParams->Amplitude_V(), amplitude);
    ToOgreVector3(this->data->waveParams->Wavenumber_V(), wavenumber);
    ToOgreVector3(this->data->waveParams->AngularFrequency_V(), omega);
    ToOgreVector3(this->data->waveParams->Steepness_V(), steepness);
    ToOgreVector2(this->data->waveParams->Direction_V(), dir0, dir1, dir2);

    // These parameters are updated on initialisation
    auto& visual = *this->data->visual;
#if 0
    visual.SetMaterialShaderParam(
      "amplitude", shaderType, Ogre::StringConverter::toString(amplitude));
    visual.SetMaterialShaderParam(
      "wavenumber", shaderType, Ogre::StringConverter::toString(wavenumber));
    visual.SetMaterialShaderParam(
      "omega", shaderType, Ogre::StringConverter::toString(omega));
    visual.SetMaterialShaderParam(
      "steepness", shaderType, Ogre::StringConverter::toString(steepness));
    visual.SetMaterialShaderParam(
      "dir0", shaderType, Ogre::StringConverter::toString(dir0));
    visual.SetMaterialShaderParam(
      "dir1", shaderType, Ogre::StringConverter::toString(dir1));
    visual.SetMaterialShaderParam(
      "dir2", shaderType, Ogre::StringConverter::toString(dir2));
#else
    rendering::SetMaterialShaderParam(visual,
      "amplitude", shaderType, Ogre::StringConverter::toString(amplitude));
    rendering::SetMaterialShaderParam(visual,
      "wavenumber", shaderType, Ogre::StringConverter::toString(wavenumber));
    rendering::SetMaterialShaderParam(visual,
      "omega", shaderType, Ogre::StringConverter::toString(omega));
    rendering::SetMaterialShaderParam(visual,
      "steepness", shaderType, Ogre::StringConverter::toString(steepness));
    rendering::SetMaterialShaderParam(visual,
      "dir0", shaderType, Ogre::StringConverter::toString(dir0));
    rendering::SetMaterialShaderParam(visual,
      "dir1", shaderType, Ogre::StringConverter::toString(dir1));
    rendering::SetMaterialShaderParam(visual,
      "dir2", shaderType, Ogre::StringConverter::toString(dir2));
#endif
  }

} // namespace asv
