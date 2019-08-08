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

#include <functional>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>
#include <ignition/math/Pose3.hh>
#include "vrx_gazebo/camera_control_plugin.hh"

/////////////////////////////////////////////
CameraControlPlugin::~CameraControlPlugin()
{
  this->connections.clear();
  this->userCam.reset();
}

/////////////////////////////////////////////
/// \brief Called after the plugin has been constructed.
void CameraControlPlugin::Load(int /*_argc*/, char ** /*_argv*/)
// void CameraControlPlugin::Load()
{
  gzerr << "CameraControlPlugin loaded" << std::endl;
  this->connections.push_back(
    gazebo::event::Events::ConnectPreRender(
      std::bind(&CameraControlPlugin::Update, this)));
}

/////////////////////////////////////////////
// \brief Called once after Load
void CameraControlPlugin::Init()
{
}

/////////////////////////////////////////////
/// \brief Called every PreRender event. See the Load function.
void CameraControlPlugin::Update()
{
  if (!this->userCam)
  {
    // Get a pointer to the active user camera
    this->userCam = gazebo::gui::get_active_camera();
  }

  // Get scene pointer
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();

  // Wait until the scene is initialized.
  if (!scene || !scene->Initialized())
    return;

  static int i = 0;
  if (i == 0)
  {
    ++i;
    ignition::math::Pose3d p(20, 20, 0, 0, 0, 0);
    this->userCam->MoveToPosition(p, 10.0);
  }
}

// Register this plugin with the simulator
//GZ_REGISTER_GUI_PLUGIN(CameraControlPlugin)
GZ_REGISTER_SYSTEM_PLUGIN(CameraControlPlugin)
