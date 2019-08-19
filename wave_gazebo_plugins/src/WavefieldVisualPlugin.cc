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
    public: Ogre::RenderTarget *renderTarget;
    public: Ogre::Camera *camera;
    public: Ogre::SceneNode *planeNode;
    public: gazebo::rendering::ScenePtr scene;
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
    // Only load plugin once, even though the model.xacro asks for 2
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
    this->data->scene = _visual->GetScene();

    // Create Plane for reflection texture
    this->data->plane = new Ogre::MovablePlane("Plane");
    this->data->plane->d = 0;
    this->data->plane->normal = Ogre::Vector3::UNIT_Z;
    Ogre::MeshManager::getSingleton().createPlane(
      "PlaneMesh",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      *(this->data->plane),
      512, 512, 1, 1,
      true,
      1, 1, 1,
      Ogre::Vector3::UNIT_Y);
    (this->data->planeEntity = this->data->scene->OgreSceneManager()->
     createEntity("PlaneMesh"));
    (this->data->planeNode = this->data->scene->OgreSceneManager()->
     getRootSceneNode()->createChildSceneNode());
    this->data->planeNode->attachObject(this->data->planeEntity);

    // QUESTION: Create render texture, if I give it the same name as
    // the texture in scripts/waves.material, it would not work for some reason
    Ogre::TexturePtr rttTexture =
      Ogre::TextureManager::getSingleton().createManual(
        "mytexture2",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        512, 512,
        0,
        Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);
    this->data->renderTarget = rttTexture->getBuffer()->getRenderTarget();

    // Setup camera
    this->data->camera = this->data->scene->GetUserCamera(0)->OgreCamera();

    // Setup render texture
    this->data->renderTarget->addViewport(this->data->camera);
    this->data->renderTarget->getViewport(0)->setClearEveryFrame(true);
    (this->data->renderTarget->getViewport(0)->
     setBackgroundColour(Ogre::ColourValue::Black));

    Ogre::MaterialPtr renderMaterial =
      Ogre::MaterialManager::getSingleton().create(
        "mymat",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    Ogre::TextureUnitState* t = (renderMaterial->getTechnique(0)->getPass(0)->
      createTextureUnitState("mytexture2"));

    t->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    t->setProjectiveTexturing(true, this->data->camera);
    this->data->renderTarget->addListener(this);

    // Camera reflection and clip plane setup
    this->data->camera->enableReflection(this->data->plane);
    this->data->camera->enableCustomNearClipPlane(this->data->plane);
    this->data->planeEntity->setMaterialName("mymat");

    // Show rendertexture on oceanwaves, not well scaled or positioned
    // this->data->visual->SetMaterial("mymat");

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

  void WavefieldVisualPlugin::preRenderTargetUpdate(
      const Ogre::RenderTargetEvent& rte)
  {
    if (this->data->planeEntity)
    {
      this->data->planeEntity->setVisible(false);
    }
    if (this->data->camera)
    {
      this->data->camera->enableReflection(this->data->plane);
      this->data->camera->enableCustomNearClipPlane(this->data->plane);
    }
  }

  void WavefieldVisualPlugin::postRenderTargetUpdate(
      const Ogre::RenderTargetEvent& rte)
  {
    if (this->data->planeEntity)
    {
      this->data->planeEntity->setVisible(true);
    }
    if (this->data->camera)
    {
      this->data->camera->disableReflection();
      this->data->camera->disableCustomNearClipPlane();
    }
  }
}
