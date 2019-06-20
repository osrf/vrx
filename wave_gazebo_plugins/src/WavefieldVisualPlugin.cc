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

#include <memory>
#include <thread>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/rendering/ogre_gazebo.h"

#include "wave_gazebo_plugins/WavefieldVisualPlugin.hh"
#include "wave_gazebo_plugins/Gazebo.hh"
#include "wave_gazebo_plugins/Wavefield.hh"
#include "wave_gazebo_plugins/Utilities.hh"

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
    for (size_t i = 0; i < _v.size(); ++i)
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
    for (size_t i = 0; i < _v.size(); ++i)
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

    /// \brief Event based connections.
    public: event::ConnectionPtr connection;
  };

///////////////////////////////////////////////////////////////////////////////
// WavefieldVisualPlugin

  WavefieldVisualPlugin::~WavefieldVisualPlugin()
  {
    // Clean up.
    this->data->waveParams.reset();

    // Reset connections and transport.
    this->data->connection.reset();
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
    // @DEBUG_INFO
    // std::thread::id threadId = std::this_thread::get_id();
    // gzmsg << "Load WavefieldVisualPlugin [thread: "
    //       << threadId << "]" << std::endl;

    // Capture visual and plugin SDF
    GZ_ASSERT(_visual != nullptr, "Visual must not be null");
    GZ_ASSERT(_sdf != nullptr, "SDF Element must not be null");

    // Capture the visual and sdf.
    this->data->visual = _visual;
    this->data->sdf = _sdf;

    // Process SDF Parameters
    #if GAZEBO_MAJOR_VERSION >= 8
      gzmsg << "WavefieldVisualPlugin <" << _visual->Name()
            << ">: Loading WaveParamaters from SDF" <<  std::endl;
    #else
      gzmsg << "WavefieldVisualPlugin <" << _visual->GetName()
            << ">: Loading WaveParamaters from SDF" <<  std::endl;
    #endif
    this->data->isStatic = Utilities::SdfParamBool(*_sdf, "static", false);
    this->data->waveParams.reset(new WaveParameters());
    if (_sdf->HasElement("wave"))
    {
      gzmsg << "Found <wave> tag" << std::endl;
      sdf::ElementPtr sdfWave = _sdf->GetElement("wave");
      this->data->waveParams->SetFromSDF(*sdfWave);
    }
    else
    {
      gzerr << "Missing <wave> tag" << std::endl;
    }

    // @DEBUG_INFO
    this->data->waveParams->DebugPrint();

    // Bind the update method to ConnectPreRender events
    this->data->connection = event::Events::ConnectPreRender(
        std::bind(&WavefieldVisualPlugin::OnUpdate, this));
  }

  void WavefieldVisualPlugin::Init()
  {
    // @DEBUG_INFO
    // std::thread::id threadId = std::this_thread::get_id();
    // gzmsg << "Init WavefieldVisualPlugin [thread: "
    //       << threadId << "]" << std::endl;

    if (!this->data->isInitialised)
    {
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
    // @DEBUG_INFO
    // gzmsg << "Reset WavefieldVisualPlugin" << std::endl;
  }

  void WavefieldVisualPlugin::OnUpdate()
  {
    if (!this->data->isStatic && !this->data->paused)
    {
#if 0
      this->data->visual->SetMaterialShaderParam(
        "time", shaderType, std::to_string(simTime));
#else
      auto simTime = this->data->visual->GetScene()->SimTime();
      rendering::SetMaterialShaderParam(*this->data->visual,
        "time", "vertex",
        std::to_string(static_cast<float>(simTime.Double())));
#endif
    }
  }

  void WavefieldVisualPlugin::SetShaderParams()
  {
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
      "Nwaves", shaderType, std::to_string(this->data->waveParams->Number()));
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
    float tau = this->data->waveParams->Tau();
    rendering::SetMaterialShaderParam(visual,
      "tau", shaderType, Ogre::StringConverter::toString(tau));
#endif
  }
}
