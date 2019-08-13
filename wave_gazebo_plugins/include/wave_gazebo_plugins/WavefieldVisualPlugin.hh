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

/// \file WavefieldVisualPlugin.hh
/// \brief This file defines a Gazebo VisualPlugin used to render
/// a wave field and keep it synchronised with the model used in
/// the physics engine.

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_VISUAL_PLUGIN_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_VISUAL_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>

#include <memory>

namespace asv
{

///////////////////////////////////////////////////////////////////////////////
// WavefieldVisualPlugin

  /// \internal
  /// \brief Class to hold private data for WavefieldModelPlugin.
  class WavefieldVisualPluginPrivate;

  class GZ_RENDERING_VISIBLE WavefieldVisualPlugin : public gazebo::VisualPlugin
  {
    /// \brief Destructor.
    public: virtual ~WavefieldVisualPlugin();

    /// \brief Constructor.
    public: WavefieldVisualPlugin();

    /// \brief Load the plugin.
    public: virtual void Load(
      gazebo::rendering::VisualPtr _visual,
      sdf::ElementPtr _sdf);

    /// internal
    /// \brief Called every PreRender event.
    private: void OnUpdate();

    private: void SetRenderTarget(Ogre::RenderTarget *_target);

    private: void UpdateFOV();

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<WavefieldVisualPluginPrivate> data;
  };
}

#endif
