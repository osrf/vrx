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

#include <algorithm>
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
#include "gazebo/rendering/RTShaderSystem.hh"
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/UserCamera.hh>
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/sensors/SensorsIface.hh"

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

    /// \brief The visual's name
    public: std::string visual_name;

    /// \brief The wavefield visual plugin SDF.
    public: sdf::ElementPtr sdf;

    /// \brief The wavefield parameters.
    public: std::shared_ptr<WaveParameters> waveParams;

    /// \brief Do not update visual if 'true', [false].
    public: bool isStatic;

    /// \brief Ratio between shallow water color and refraction color to use
    ///        In [0, 1], where 0 is no refraction and 1 is maximum refraction
    public: double shallowRefractRatio;

    /// \brief Ratio between environment color and reflection color to use
    ///        In [0, 1], where 0 is no reflection and 1 is maximum reflection
    public: double envReflectRatio;

    /// \brief World stats.
    public: double simTime, realTime, pauseTime;
    public: bool paused;

    /// \brief Prevent multiple calls to Init loading visuals twice...
    public: bool isInitialised;

    // OGRE objects for reflection/refraction
    public: gazebo::rendering::ScenePtr scene;
    public: Ogre::Entity* planeEntity;
    public: Ogre::Plane planeUp;
    public: Ogre::Plane planeDown;
    public: Ogre::ColourValue backgroundColor;
    public: Ogre::MaterialPtr material;

    // To be replaced by vectors soon
    public: Ogre::Camera *camera;
    public: Ogre::TexturePtr rttReflectionTexture;
    public: Ogre::TexturePtr rttRefractionTexture;
    public: Ogre::RenderTarget *reflectionRt;
    public: Ogre::RenderTarget *refractionRt;

    // Vectors of OGRE objects
    public: std::vector<rendering::CameraPtr> cameras;
    public: std::vector<Ogre::TexturePtr> rttReflectionTextures;
    public: std::vector<Ogre::TexturePtr> rttRefractionTextures;
    public: std::vector<Ogre::RenderTarget*> reflectionRts;
    public: std::vector<Ogre::RenderTarget*> refractionRts;

    /// \brief Event based connections.
    public: event::ConnectionPtr preRenderConnection;
    public: event::ConnectionPtr renderConnection;
  };

///////////////////////////////////////////////////////////////////////////////
// WavefieldVisualPlugin

  WavefieldVisualPlugin::~WavefieldVisualPlugin()
  {
    // Clean up.
    this->data->waveParams.reset();

    // Reset connections and transport.
    this->data->preRenderConnection.reset();
    this->data->renderConnection.reset();
  }

  WavefieldVisualPlugin::WavefieldVisualPlugin() :
    VisualPlugin(),
    RenderTargetListener(),
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
      this->data->visual_name = _visual->Name();
    #else
      this->data->visual_name = _visual->GetName();
    #endif

    gzmsg << "WavefieldVisualPlugin <" << this->data->visual_name
          << ">: Loading WaveParamaters from SDF" <<  std::endl;

    this->data->isStatic = Utilities::SdfParamBool(*_sdf, "static", false);

    // Read refraction and reflection ratios
    this->data->shallowRefractRatio =
      Utilities::SdfParamDouble(*_sdf, "shallowRefractRatio", 0.2);
    this->data->envReflectRatio =
      Utilities::SdfParamDouble(*_sdf, "envReflectRatio", 0.2);

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

    // Setup reflection refraction
    this->SetupReflectionRefraction();

    // Bind the update method to ConnectPreRender events
    this->data->preRenderConnection = event::Events::ConnectPreRender(
        std::bind(&WavefieldVisualPlugin::OnPreRender, this));
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

  void WavefieldVisualPlugin::OnPreRender()
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

      std::vector<rendering::CameraPtr> new_cameras = this->NewCameras();
      gzerr << "Number of camera sensors: " << new_cameras.size() << std::endl;
    }
  }

  void WavefieldVisualPlugin::OnRender()
  {
    if (!this->data->camera || !this->data->reflectionRt ||
        !this->data->refractionRt)
      return;

    // Update reflection/refraction
    this->data->reflectionRt->update();
    this->data->refractionRt->update();
  }

  std::vector<rendering::CameraPtr> WavefieldVisualPlugin::NewCameras()
  {
    std::vector<rendering::CameraPtr> retVal;

    sensors::Sensor_V all_sensors = sensors::SensorManager::Instance()->GetSensors();
    for (sensors::SensorPtr sensor : all_sensors)
    {
      // Check if sensor is a camera and can be casted
      if (sensor->Type().compare("camera") != 0)
      {
        continue;
      }
      sensors::CameraSensorPtr c = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
      if (!c)
      {
        continue;
      }

      // Add new cameras
      rendering::CameraPtr camera = c->Camera();
      if(std::find(this->data->cameras.begin(), this->data->cameras.end(), camera) == this->data->cameras.end())
      {
        retVal.push_back(camera);
      }
    }

    return retVal;
  }
  void WavefieldVisualPlugin::SetupReflectionRefraction()
  {
    // OGRE setup
    this->data->scene = this->data->visual->GetScene();

    // Only load the visual plugin in gzclient for now.
    if (!this->data->scene->EnableVisualizations())
      return;

    // Setup planeEntity
    Ogre::SceneNode *ogreNode = this->data->visual->GetSceneNode();
    this->data->planeEntity =
        dynamic_cast<Ogre::Entity *>(ogreNode->getAttachedObject(0));
    if (!this->data->planeEntity)
    {
      gzerr << "No plane entity found" << std::endl;
      return;
    }

    // Render water later for proper rendering of propeller
    this->data->planeEntity->setRenderQueueGroup(this->data->planeEntity->
                                                 getRenderQueueGroup()+1);

    // Create clipping planes to hide objects
    this->data->planeUp = Ogre::Plane(Ogre::Vector3::UNIT_Z, 0);  // TODO: change location depending on ocean location and orientation
    this->data->planeDown = Ogre::Plane(-Ogre::Vector3::UNIT_Z, 0);

    // Get background color
    this->data->backgroundColor =
        rendering::Conversions::Convert(this->data->scene->BackgroundColor());

    // Give material the new textures
    this->data->material =
      Ogre::MaterialManager::getSingleton().getByName(this->data->visual->
                                                      GetMaterialName());

    // Bind the update method to ConnectRender events
    this->data->renderConnection = event::Events::ConnectRender(
        std::bind(&WavefieldVisualPlugin::OnRender, this));

    // Set reflection/refraction parameters
    rendering::SetMaterialShaderParam(*this->data->visual,
      "shallowRefractRatio", "fragment",
      std::to_string(static_cast<float>(this->data->shallowRefractRatio)));
    rendering::SetMaterialShaderParam(*this->data->visual,
      "envReflectRatio", "fragment",
      std::to_string(static_cast<float>(this->data->envReflectRatio)));

    // Setup camera
    rendering::CameraPtr userCamera = this->data->scene->GetUserCamera(0);

    if (!userCamera)
    {
      gzerr << "User camera not found" << std::endl;
      return;
    }

    this->CreateReflectionRefractionTextures(userCamera);
  }

  void WavefieldVisualPlugin::CreateReflectionRefractionTextures(rendering::CameraPtr camera)
  {
    // Create reflection texture
    Ogre::TexturePtr rttReflectionTexture =
      Ogre::TextureManager::getSingleton().createManual(
        this->data->visual_name + "_" + camera->ScopedName() + "_reflection",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        512, 512,
        0,
        Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);

    // Create refraction texture
    Ogre::TexturePtr rttRefractionTexture =
      Ogre::TextureManager::getSingleton().createManual(
        this->data->visual_name + "_" + camera->ScopedName() + "_refraction",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        512, 512,
        0,
        Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);

    // Setup reflection render target
    Ogre::RenderTarget *reflectionRt =
        rttReflectionTexture->getBuffer()->getRenderTarget();
    reflectionRt->setAutoUpdated(false);
    Ogre::Viewport *reflVp =
        reflectionRt->addViewport(camera->OgreCamera());
    reflVp->setClearEveryFrame(true);
    reflVp->setOverlaysEnabled(false);
    reflVp->setBackgroundColour(this->data->backgroundColor);
    reflVp->setVisibilityMask(GZ_VISIBILITY_ALL &
        ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
    rendering::RTShaderSystem::AttachViewport(reflVp, this->data->scene);
    reflectionRt->addListener(this);

    // Setup refraction render target
    Ogre::RenderTarget *refractionRt =
        rttRefractionTexture->getBuffer()->getRenderTarget();
    refractionRt->setAutoUpdated(false);
    Ogre::Viewport *refrVp =
        refractionRt->addViewport(camera->OgreCamera());
    refrVp->setClearEveryFrame(true);
    refrVp->setOverlaysEnabled(false);
    refrVp->setBackgroundColour(this->data->backgroundColor);
    refrVp->setVisibilityMask(GZ_VISIBILITY_ALL &
        ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
    rendering::RTShaderSystem::AttachViewport(refrVp, this->data->scene);
    refractionRt->addListener(this);

    this->data->cameras.push_back(camera);
    this->data->rttReflectionTextures.push_back(rttReflectionTexture);
    this->data->rttRefractionTextures.push_back(rttRefractionTexture);
    this->data->reflectionRts.push_back(reflectionRt);
    this->data->refractionRts.push_back(refractionRt);

    this->data->camera = camera->OgreCamera();
    this->data->rttReflectionTexture = rttReflectionTexture;
    this->data->rttRefractionTexture = rttRefractionTexture;
    this->data->reflectionRt = reflectionRt;
    this->data->refractionRt = refractionRt;

    Ogre::TextureUnitState *reflectTex =
        this->data->material->getTechnique(0)->getPass(0)->getTextureUnitState(2);
    reflectTex->setTexture(rttReflectionTexture);
    Ogre::TextureUnitState *refractTex =
        this->data->material->getTechnique(0)->getPass(0)->getTextureUnitState(3);
    refractTex->setTexture(rttRefractionTexture);
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

  void WavefieldVisualPlugin::preRenderTargetUpdate(
      const Ogre::RenderTargetEvent& rte)
  {
    if (!this->data->camera)
      return;

    if (this->data->planeEntity)
    {
      this->data->planeEntity->setVisible(false);
    }

    // Reflection: hide objects below water
    if (rte.source == this->data->reflectionRt)
    {
      this->data->camera->enableReflection(this->data->planeUp);
      this->data->camera->enableCustomNearClipPlane(this->data->planeUp);
    }
    // Refraction: hide objects above water
    else
    {
      this->data->visual->SetVisible(false);
      this->data->camera->enableCustomNearClipPlane(this->data->planeDown);
    }
  }

  void WavefieldVisualPlugin::postRenderTargetUpdate(
      const Ogre::RenderTargetEvent& rte)
  {
    if (!this->data->camera)
      return;

    if (this->data->planeEntity)
    {
      this->data->planeEntity->setVisible(true);
    }

    // Reflection: unhide objects below water
    if (rte.source == this->data->reflectionRt)
    {
      this->data->camera->disableReflection();
      this->data->camera->disableCustomNearClipPlane();
    }
    // Refraction: unhide objects above water
    else
    {
      this->data->camera->disableCustomNearClipPlane();
    }
  }
}
