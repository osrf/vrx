/*
 * Copyright (C) 2019  Rhys Mainwaring
 * * Licensed under the Apache License, Version 2.0 (the "License");
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
#include <string>

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

#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/ogre_gazebo.h"

#include "wave_gazebo_plugins/WavefieldVisualPlugin.hh"
#include "wave_gazebo_plugins/Gazebo.hh"
#include "wave_gazebo_plugins/Wavefield.hh"
#include "wave_gazebo_plugins/Utilities.hh"

using namespace gazebo;

namespace asv
{
  GZ_REGISTER_VISUAL_PLUGIN(WavefieldVisualPlugin)

// WavefieldVisualPluginPrivate

  /// \internal
  /// \brief Private data for the WavefieldVisualPlugin
  class WavefieldVisualPluginPrivate
  {
    /// \brief The visual containing this plugin.
    public: rendering::VisualPtr visual;

    /// \brief The wavefield visual plugin SDF.
    public: sdf::ElementPtr sdf;

    /// \brief Event based connections.
    public: event::ConnectionPtr connection;

    // OGRE 
    public: Ogre::Texture *renderTexture;
    public: Ogre::RenderTarget *renderTarget;
    public: Ogre::Camera *camera;
    public: Ogre::Root *root;
    public: Ogre::SceneManager* mSceneMgr;
    public: Ogre::SceneNode *cameraNode;
    public: Ogre::SceneNode *sceneNode;
    public: Ogre::Viewport *viewport;
    public: gazebo::rendering::ScenePtr scene;
    public: Ogre::Rectangle2D* miniscreen;
    public: Ogre::MovablePlane* plane;
    public: Ogre::Entity* planeEntity;
  };

///////////////////////////////////////////////////////////////////////////////
// WavefieldVisualPlugin

  WavefieldVisualPlugin::~WavefieldVisualPlugin()
  {
    // Reset connections and transport.
    this->data->connection.reset();
  }

  WavefieldVisualPlugin::WavefieldVisualPlugin() :
    VisualPlugin(),
    RenderTargetListener(),
    data(new WavefieldVisualPluginPrivate())
  {
  }

  void WavefieldVisualPlugin::Load(
    rendering::VisualPtr _visual,
    sdf::ElementPtr _sdf)
  {
    // Only load plugin once
    static int i = 0;
    i++;
    if (i == 2) { return; }

    // Capture visual and plugin SDF
    GZ_ASSERT(_visual != nullptr, "Visual must not be null");
    GZ_ASSERT(_sdf != nullptr, "SDF Element must not be null");

    // Capture the visual and sdf.
    this->data->visual = _visual;
    this->data->sdf = _sdf;

    // OGRE setup
    this->data->root = Ogre::Root::getSingletonPtr();
    this->data->scene = _visual->GetScene();

    // Setup camera from user camera
    Ogre::Camera* user_camera = this->data->scene->GetUserCamera(0)->OgreCamera();
    this->data->camera = this->data->scene->OgreSceneManager()->createCamera("reflectCam");
    //this->data->camera = user_camera;
    Ogre::Camera *mCamera = this->data->camera;
    mCamera->setPosition(user_camera->getPosition());
    mCamera->setOrientation(user_camera->getOrientation());
    mCamera->setNearClipDistance(user_camera->getNearClipDistance());
    mCamera->setFarClipDistance(user_camera->getFarClipDistance());
    mCamera->setAspectRatio(user_camera->getAspectRatio());

    // TESTING TUTORIAL setup
    Ogre::MovablePlane* mPlane(0);
    Ogre::Entity* mPlaneEntity(0);
    Ogre::SceneNode* mPlaneNode(0);
    Ogre::Rectangle2D* mMiniScreen(0);

    // Add lighting
    this->data->scene->OgreSceneManager()->setAmbientLight(Ogre::ColourValue(0.2, 0.2, 0.2));
    Ogre::Light* light = this->data->scene->OgreSceneManager()->createLight("mylight" + std::to_string(i));
    light->setPosition(20, 80, 50);

    // Create render texture
    Ogre::TexturePtr rttTexture =
      Ogre::TextureManager::getSingleton().createManual(
        "mytexture",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, 
        Ogre::TEX_TYPE_2D, 
        512, 512, 
        0, 
        Ogre::PF_R8G8B8, 
        Ogre::TU_RENDERTARGET);
    Ogre::RenderTexture* renderTexture = rttTexture->getBuffer()->getRenderTarget();

    // Setup render texture
    renderTexture->addViewport(mCamera);
    renderTexture->getViewport(0)->setClearEveryFrame(true);
    renderTexture->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
    renderTexture->getViewport(0)->setOverlaysEnabled(false);
    this->data->renderTarget = renderTexture;
    renderTexture->writeContentsToFile("/home/tylerlum/start.png");
    Ogre::MaterialPtr renderMaterial =
      Ogre::MaterialManager::getSingleton().create(
        "mymat",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    Ogre::TextureUnitState* t = renderMaterial->getTechnique(0)->getPass(0)->createTextureUnitState("mytexture");
    renderMaterial->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    //t->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    //t->setProjectiveTexturing(true, mCamera);
    renderTexture->update();
    renderTexture->addListener(this);

    // Create Plane for reflection texture
    this->data->plane = new Ogre::MovablePlane("Plane" + std::to_string(i));
    mPlane = this->data->plane;
    mPlane->d = 1;
    mPlane->normal = Ogre::Vector3::UNIT_Y;
    Ogre::MeshManager::getSingleton().createPlane(
      "PlaneMesh" + std::to_string(i),
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      *mPlane,
      100, 100, 1, 1,
      true,
      1, 1, 1,
      Ogre::Vector3::UNIT_Z);
    //mCamera->enableReflection(this->data->plane);
    //mCamera->enableCustomNearClipPlane(this->data->plane);

    // Create Plane entity with correct material and texture
    this->data->planeEntity = this->data->scene->OgreSceneManager()->createEntity("PlaneMesh" + std::to_string(i));
    mPlaneEntity = this->data->planeEntity;
    mPlaneEntity->setMaterialName("mymat");
    mPlaneNode = this->data->scene->OgreSceneManager()->getRootSceneNode()->createChildSceneNode();
    mPlaneNode->attachObject(mPlaneEntity);

    // Create miniscreen and node
    this->data->miniscreen = new Ogre::Rectangle2D(true);
    mMiniScreen = this->data->miniscreen;

    mMiniScreen->setCorners(.5, 1.0, 1.0, .5);
    mMiniScreen->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
    Ogre::SceneNode* miniScreenNode =
      this->data->scene->OgreSceneManager()->getRootSceneNode()->createChildSceneNode();
    miniScreenNode->attachObject(mMiniScreen);
    mMiniScreen->setMaterial("mymat");

    // Bind the update method to ConnectPreRender events
    this->data->connection = event::Events::ConnectRender(
        std::bind(&WavefieldVisualPlugin::OnUpdate, this));
  }

  void WavefieldVisualPlugin::OnUpdate()
  {
    if (this->data->renderTarget)
    {
      this->data->renderTarget->update();
    }
  }

  void WavefieldVisualPlugin::preRenderTargetUpdate(const Ogre::RenderTargetEvent& rte)
  {
    if (this->data->planeEntity)
    {
      this->data->planeEntity->setVisible(false);
    }
    //if (this->data->camera)
    //{
    //  this->data->camera->enableReflection(this->data->plane);
    //}
    if (this->data->miniscreen)
    {
      this->data->miniscreen->setVisible(false);
    }
  }
  
  void WavefieldVisualPlugin::postRenderTargetUpdate(const Ogre::RenderTargetEvent& rte)
  {
    if (this->data->planeEntity)
    {
      this->data->planeEntity->setVisible(true);
    }
    //if (this->data->camera)
    //{
    //  this->data->camera->disableReflection();
    //}
    if (this->data->miniscreen)
    {
      this->data->miniscreen->setVisible(true);
    }
  }
}
