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

#ifndef VRX_GAZEBO_CAMERA_CONTROL_PLUGIN_HH_
#define VRX_GAZEBO_CAMERA_CONTROL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
//#include <gazebo/gui/GuiPlugin.hh>

/// \brief ToDo.
// class CameraControlPlugin : public gazebo::GUIPlugin
class CameraControlPlugin : public gazebo::SystemPlugin
{
  /// \brief Constructor.
  public: CameraControlPlugin() = default;

  /// \brief Destructor.
  public: virtual ~CameraControlPlugin();

  // Documentation inherited.
  public: void Load(int /*_argc*/, char ** /*_argv*/);
  //public: void Load();

  // Documentation inherited.
  private: void Init();

  // Documentation inherited.
  private: void Update();

  /// Pointer the user camera.
  private: gazebo::rendering::UserCameraPtr userCam;

  /// All the event connections.
  private: std::vector<gazebo::event::ConnectionPtr> connections;
};

#endif
