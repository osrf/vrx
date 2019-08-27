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
#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/UserCamera.hh"
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
    public: WavefieldVisualPluginPrivate() :
            planeUp("planeUp"),
            planeDown("planeDown")
            {}

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
    public: double simTime;

    /// \brief Prevent multiple calls to Init loading visuals twice...
    public: bool isInitialised;

    // OGRE objects for reflection/refraction
    public: gazebo::rendering::ScenePtr scene;
    public: Ogre::Entity* oceanEntity;
    public: Ogre::MovablePlane planeUp;
    public: Ogre::MovablePlane planeDown;
    public: Ogre::ColourValue backgroundColor;
    public: Ogre::MaterialPtr material;
    public: Ogre::TextureUnitState *reflectTex;
    public: Ogre::TextureUnitState *refractTex;

    // Vectors of OGRE objects
    public: std::vector<Ogre::Camera*> cameras;
    public: std::vector<Ogre::TexturePtr> rttReflectionTextures;
    public: std::vector<Ogre::TexturePtr> rttRefractionTextures;
    public: std::vector<Ogre::RenderTarget*> reflectionRts;
    public: std::vector<Ogre::RenderTarget*> refractionRts;

    public: bool rttUpdate = false;

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
    // Update clip plane pose (in case the ocean moves, may need to optimize)
    Ogre::Vector3 oceanPosition(this->data->visual->WorldPose().Pos().X(),
                                this->data->visual->WorldPose().Pos().Y(),
                                this->data->visual->WorldPose().Pos().Z());

    Ogre::Quaternion oceanRotation(this->data->visual->WorldPose().Rot().W(),
                                   this->data->visual->WorldPose().Rot().X(),
                                   this->data->visual->WorldPose().Rot().Y(),
                                   this->data->visual->WorldPose().Rot().Z());
    Ogre::Vector3 oceanNormal = oceanRotation * Ogre::Vector3::UNIT_Z;
    this->data->planeUp.redefine(oceanNormal, oceanPosition);
    this->data->planeDown.redefine(-oceanNormal, oceanPosition);

    // User cam setup in gzclient
    if (this->data->scene->EnableVisualizations())
    {
      // Get user cam
      rendering::UserCameraPtr userCamera = this->data->scene->GetUserCamera(0);

      // If user cam not already in cameras
      if (std::find(this->data->cameras.begin(), this->data->cameras.end(), userCamera->OgreCamera())
         == this->data->cameras.end())
      {
        // Add listener for user cam
        userCamera->OgreViewport()->getTarget()->addListener(this);
        // Create rtts for usercam
        this->CreateReflectionRefractionTextures(userCamera->OgreCamera());
      }
    }

    // Camera sensor setup in gzserver
    else
    {
      // Get new cameras
      std::vector<rendering::CameraPtr> newCameras = this->NewCameras();
      int p = 0;
      for (rendering::CameraPtr c : newCameras)
      {
        p++;
        // Add listener for camera sensor
        Ogre::Texture *rt = c->RenderTexture();
        if (!rt) { gzerr << p << " !rt" << std::endl; return; }
        rt->getBuffer()->getRenderTarget()->addListener(this);

        // Create rtts for usercam
        this->CreateReflectionRefractionTextures(c->OgreCamera());
      }
    }

    // Create moving ocean waves
    if (!this->data->isStatic)
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

  void WavefieldVisualPlugin::OnRender()
  {
    std::string path = "";
    if (char* home_dir = std::getenv("HOME"))
    {
      path += std::string(home_dir);
    }

    // Reflection
    for (unsigned int i = 0; i < this->data->reflectionRts.size(); ++i)
    {
      Ogre::RenderTarget* rt = this->data->reflectionRts.at(i);
      rt->update();
      rt->writeContentsToFile(path + "/" + this->data->cameras.at(i)->getName()
                              + "_reflection.png");
    }

    // Refraction
    for (unsigned int i = 0; i < this->data->refractionRts.size(); ++i)
    {
      Ogre::RenderTarget* rt = this->data->refractionRts.at(i);
      rt->update();
      rt->writeContentsToFile(path + "/" + this->data->cameras.at(i)->getName()
                              + "_refraction.png");
    }
  }

  void WavefieldVisualPlugin::SetupReflectionRefraction()
  {
    // OGRE setup
    this->data->scene = this->data->visual->GetScene();

    // Setup oceanEntity
    Ogre::SceneNode *ogreNode = this->data->visual->GetSceneNode();
    this->data->oceanEntity =
        dynamic_cast<Ogre::Entity *>(ogreNode->getAttachedObject(0));
    if (!this->data->oceanEntity)
    {
      gzerr << "No plane entity found" << std::endl;
      return;
    }

    // Render water later for proper rendering of propeller
    this->data->oceanEntity->setRenderQueueGroup(this->data->oceanEntity->
                                                 getRenderQueueGroup()+1);

    // Create clipping planes to hide objects, default pose
    this->data->planeUp = Ogre::MovablePlane(Ogre::Vector3::UNIT_Z,
                                             Ogre::Vector3::ZERO);
    this->data->planeDown = Ogre::MovablePlane(-Ogre::Vector3::UNIT_Z,
                                               Ogre::Vector3::ZERO);

    // Get background color
    this->data->backgroundColor =
        rendering::Conversions::Convert(this->data->scene->BackgroundColor());

    // Get material to give new textures
    this->data->material =
      Ogre::MaterialManager::getSingleton().getByName(this->data->visual->
                                                      GetMaterialName());
    this->data->reflectTex =
        (this->data->material->getTechnique(0)->getPass(0)->
         getTextureUnitState(2));
    this->data->refractTex =
        (this->data->material->getTechnique(0)->getPass(0)->
         getTextureUnitState(3));

    // Set reflection/refraction parameters
    rendering::SetMaterialShaderParam(*this->data->visual,
      "shallowRefractRatio", "fragment",
      std::to_string(static_cast<float>(this->data->shallowRefractRatio)));
    rendering::SetMaterialShaderParam(*this->data->visual,
      "envReflectRatio", "fragment",
      std::to_string(static_cast<float>(this->data->envReflectRatio)));


    // Bind the update method to ConnectRender events
    // this->data->renderConnection = event::Events::ConnectRender(
        // std::bind(&WavefieldVisualPlugin::OnRender, this));
  }

  void WavefieldVisualPlugin::CreateReflectionRefractionTextures(Ogre::Camera*
                                                                 camera)
  {
    // Create reflection texture
    Ogre::TexturePtr rttReflectionTexture =
      Ogre::TextureManager::getSingleton().createManual(
        this->data->visual_name + "_" + camera->getName() + "_reflection",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        512, 512,
        0,
        Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);

    // Create refraction texture
    Ogre::TexturePtr rttRefractionTexture =
      Ogre::TextureManager::getSingleton().createManual(
        this->data->visual_name + "_" + camera->getName() + "_refraction",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        512, 512,
        0,
        Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);

    // Setup reflection render target
    Ogre::RenderTarget* reflectionRt =
        rttReflectionTexture->getBuffer()->getRenderTarget();
    reflectionRt->setAutoUpdated(false);
    Ogre::Viewport *reflVp =
        reflectionRt->addViewport(camera);
    reflVp->setClearEveryFrame(true);
    reflVp->setOverlaysEnabled(false);
    reflVp->setBackgroundColour(this->data->backgroundColor);
    reflVp->setVisibilityMask(GZ_VISIBILITY_ALL &
        ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
    rendering::RTShaderSystem::AttachViewport(reflVp, this->data->scene);
    reflectionRt->addListener(this);

    // Setup refraction render target
    Ogre::RenderTarget* refractionRt =
        rttRefractionTexture->getBuffer()->getRenderTarget();
    refractionRt->setAutoUpdated(false);
    Ogre::Viewport *refrVp =
        refractionRt->addViewport(camera);
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
  }

  std::vector<rendering::CameraPtr> WavefieldVisualPlugin::NewCameras()
  {
    std::vector<rendering::CameraPtr> retVal;

    sensors::Sensor_V all_sensors = (sensors::SensorManager::Instance()->
                                     GetSensors());
    for (sensors::SensorPtr sensor : all_sensors)
    {
      // Check if sensor is a camera and can be casted
      if (sensor->Type().compare("camera") != 0)
      {
        continue;
      }
      sensors::CameraSensorPtr camera =
        std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
      if (!camera)
      {
        continue;
      }

      // Add new cameras
      rendering::CameraPtr c = camera->Camera();
      if (std::find(this->data->cameras.begin(), this->data->cameras.end(), c->OgreCamera())
         == this->data->cameras.end())
      {
        retVal.push_back(c);
      }
    }

    return retVal;
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
    if (this->data->cameras.size() == 0)
      return;

    if (this->data->oceanEntity)
    {
      this->data->oceanEntity->setVisible(false);
    }

    // On Camera preupdate, update rtts first before updating camera
    if (!this->data->rttUpdate)
    {
      for (unsigned int i = 0; i < this->data->cameras.size(); ++i)
      {
        Ogre::RenderTarget *rt = this->data->cameras.at(i)->getViewport()->getTarget();
        if (rte.source->getViewport(0)->getCamera() == this->data->cameras.at(i))
        {
          this->data->rttUpdate = true;
          this->data->reflectionRts.at(i)->update();
          this->data->refractionRts.at(i)->update();
          this->data->rttUpdate = false;
          return;
        }
      }
    }

    // Reflection
    for (unsigned int i = 0; i < this->data->reflectionRts.size(); ++i)
    {
      Ogre::RenderTarget* rt = this->data->reflectionRts.at(i);
      if (rte.source == rt)
      {
        this->data->cameras.at(i)->enableReflection(this->data->planeUp);
        this->data->cameras.at(i)->enableCustomNearClipPlane(this->data->
                                                             planeUp);
        this->data->reflectTex->setTexture(this->data->
                                           rttReflectionTextures.at(i));
        return;
      }
    }

    // Refraction
    for (unsigned int i = 0; i < this->data->refractionRts.size(); ++i)
    {
      Ogre::RenderTarget* rt = this->data->refractionRts.at(i);
      if (rte.source == rt)
      {
        this->data->cameras.at(i)->enableCustomNearClipPlane(this->data->
                                                             planeDown);
        this->data->refractTex->setTexture(this->data->
                                           rttRefractionTextures.at(i));
        return;
      }
    }
  }

  void WavefieldVisualPlugin::postRenderTargetUpdate(
      const Ogre::RenderTargetEvent& rte)
  {
    if (this->data->cameras.size() == 0)
      return;

    if (this->data->oceanEntity)
    {
      this->data->oceanEntity->setVisible(true);
    }

    // Reflection
    for (unsigned int i = 0; i < this->data->reflectionRts.size(); ++i)
    {
      Ogre::RenderTarget* rt = this->data->reflectionRts.at(i);
      if (rte.source == rt)
      {
        this->data->cameras.at(i)->disableReflection();
        this->data->cameras.at(i)->disableCustomNearClipPlane();
        this->data->reflectTex->setTexture(this->data->
                                           rttReflectionTextures.at(i));
        return;
      }
    }

    // Refraction
    for (unsigned int i = 0; i < this->data->refractionRts.size(); ++i)
    {
      Ogre::RenderTarget* rt = this->data->refractionRts.at(i);
      if (rte.source == rt)
      {
        this->data->cameras.at(i)->disableCustomNearClipPlane();
        this->data->refractTex->setTexture(this->data->
                                           rttRefractionTextures.at(i));
        return;
      }
    }
  }
}
