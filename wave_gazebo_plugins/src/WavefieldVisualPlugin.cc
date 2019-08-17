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
    data(new WavefieldVisualPluginPrivate)
  {
  }

  void WavefieldVisualPlugin::Load(
    rendering::VisualPtr _visual,
    sdf::ElementPtr _sdf)
  {
    // Capture visual and plugin SDF
    GZ_ASSERT(_visual != nullptr, "Visual must not be null");
    GZ_ASSERT(_sdf != nullptr, "SDF Element must not be null");

    // Capture the visual and sdf.
    this->data->visual = _visual;
    this->data->sdf = _sdf;

    // OGRE
    this->data->root = Ogre::Root::getSingletonPtr();
    this->data->mSceneMgr = this->data->root->createSceneManager(Ogre::ST_EXTERIOR_CLOSE);
    this->data->sceneNode = this->data->mSceneMgr->getRootSceneNode()->createChildSceneNode("mycam_SceneNode");
    this->data->camera = this->data->mSceneMgr->createCamera("mycam");
    this->data->cameraNode = this->data->sceneNode->createChildSceneNode("mycam_cameraNode");
    this->data->cameraNode->attachObject(this->data->camera);
  
    this->data->cameraNode->yaw(Ogre::Degree(-90.0));
    this->data->cameraNode->roll(Ogre::Degree(-90.0));
    this->data->camera = this->data->mSceneMgr->createCamera("PlayerCam");

    this->data->camera->setPosition(Ogre::Vector3(0, 0, 80));
    this->data->camera->lookAt(Ogre::Vector3(0, 0, -300));
    this->data->camera->setNearClipDistance(5);
    this->data->scene = gazebo::rendering::get_scene("default");

    this->data->renderTexture = Ogre::TextureManager::getSingleton().createManual("reflection", "General", Ogre::TEX_TYPE_2D, 512, 512, 0, Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET).getPointer();
    this->SetRenderTarget(this->data->renderTexture->getBuffer()->getRenderTarget());
    //
    // Bind the update method to ConnectPreRender events
    this->data->connection = event::Events::ConnectRender(
        std::bind(&WavefieldVisualPlugin::OnUpdate, this));
  }

  void WavefieldVisualPlugin::OnUpdate()
  {
    //gzerr << "ON UPDATE" << std::endl;
  }

  void WavefieldVisualPlugin::SetRenderTarget(Ogre::RenderTarget *_target)
  {
    gzerr << "SET RENDER TARGET" << std::endl;
    this->data->renderTarget = _target;

    if (this->data->renderTarget)
    {
      // Setup the viewport to use the texture
      this->data->viewport = this->data->renderTarget->addViewport(this->data->camera);
      this->data->viewport->setClearEveryFrame(true);
      this->data->viewport->setShadowsEnabled(true);
      this->data->viewport->setOverlaysEnabled(false);

      //RTShaderSystem::AttachViewport(this->data->viewport, this->GetScene());

      //auto const &ignBG = this->scene->BackgroundColor();
      //this->data->viewport->setBackgroundColour(Conversions::Convert(ignBG));
      //this->data->viewport->setVisibilityMask(GZ_VISIBILITY_ALL &
      //   ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
    }
  //  if (this->data->renderTarget)
  //  {
  //    this->data->viewport = this->data->renderTarget->addViewport(this->data->camera);
  //    this->data->viewport->setClearEveryFrame(true);
  //    this->data->viewport->setShadowsEnabled(true);
  //    this->data->viewport->setOverlaysEnabled(false);
  //    this->data->viewport->setBackgroundColour(Ogre::ColourValue::Black);
  //    this->UpdateFOV();
  //  }
  }

  // void WavefieldVisualPlugin::UpdateFOV()
  // {
  //   if (this->data->viewport)
  //   {
  //     this->data->viewport->setDimensions(0, 0, 1, 1);
  //     double ratio = static_cast<double>(this->data->viewport->getActualWidth()) / static_cast<double>(this->data->viewport->getActualHeight());
  //     double hfov = this->HFOV().Radian();
  //     double vfov = 2.0 * atan(tan(hfov/2.0) / ratio);
  //   }
  // }

}
