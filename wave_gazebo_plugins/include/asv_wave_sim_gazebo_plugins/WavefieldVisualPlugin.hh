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

/// \file WavefieldVisualPlugin.hh
/// \brief This file defines a Gazebo VisualPlugin used to render
/// a wave field and keep it synchronised with the model used in the physics engine.

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

  /// \brief A Gazebo visual plugin to synchronise and control 
  /// a vertex shader rendering Gerstner waves.
  ///
  /// # Usage
  ///
  /// Add the SDF for the plugin to the <visual> element of your wave model.
  ///
  /// The SDF parameters specifying the wave are all optional, and normal
  /// use will be overridden.
  ///
  /// If this visual is loaded as part of a wave model that also contains 
  /// the plugin libWavefieldModelPlugin.so, then it will receive a response
  /// to its request for ~/wave_param and set the wave parameters to be
  /// consistent with the wave generator operating on the physics server.
  ///
  /// \code
  /// <plugin name="wavefield_visual" filename="libWavefieldVisualPlugin.so">
  ///   <static>false</static>
  ///   <wave>
  ///     <number>3</number>
  ///     <scale>1.5</scale>
  ///     <angle>0.4</angle>
  ///     <steepness>1.0</steepness>
  ///     <amplitude>0.4</amplitude>
  ///     <period>8.0</period>
  ///     <direction>1 1</direction>
  ///   </wave>
  /// </plugin>
  /// \endcode
  ///
  /// # Subscribed Topics
  ///
  /// 1. ~/reponse (gazebo::msgs::Response)
  ///   
  /// 2. ~/wave (gazebo::msgs::Param_V)
  ///
  /// 3. ~/world_stats (gazebo::msgs::WorldStatistics)
  ///
  /// 4. /marker (ignition::msgs::Marker)
  ///
  /// # Published Topics
  ///
  /// 1. ~/request (gazebo::msgs::Request)
  ///
  /// # Parameters
  ///
  /// 1. <static> (bool, default: false)
  ///   Display a static wave field if set to true.
  ///   
  /// 2. <number> (int, default: 1)
  ///   The number of component waves.
  ///
  /// 3. <scale> (double, default: 2.0)
  ///   The scale between the mean and largest / smallest component waves.
  ///
  /// 4. <angle> (double, default: 2*pi/10)
  ///   The angle between the mean wave direction and the largest / smallest component waves.
  ///
  /// 5. <steepness> (double, default: 1.0)
  ///   A parameter in [0, 1] controlling the wave steepness with 1 being steepest.
  ///
  /// 6. <amplitude> (double, default: 0.0)
  ///   The amplitude of the mean wave in [m].
  ///
  /// 7. <period> (double, default: 1.0)
  ///   The period of the mean wave in [s].
  ///
  /// 8. <phase> (double, default: 0.0)
  ///   The phase of the mean wave.
  ///
  /// 9. <direction> (Vector2D, default: (1 0))
  ///   A two component vector specifiying the direction of the mean wave.
  ///
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

    /// \brief Initialize the plugin.
    public: virtual void Init();

    /// \brief Reset the plugin.
    public: virtual void Reset();

    /// internal
    /// \brief Called every PreRender event.
    private: void OnUpdate();

    /// internal
    /// \brief Callback for gztopic "~/response" when the response type is a wave message.
    ///
    /// \param[in] _msg Response message.
    private: void OnResponse(ConstResponsePtr &_msg);

    /// internal
    /// \brief Callback for gztopic "~/wave".
    ///
    /// \param[in] _msg Wave message.
    private: void OnWaveMsg(ConstParam_VPtr &_msg);

    /// internal
    /// \brief Callback for gztopic "~/world_stats".
    ///
    /// \param[in] _msg World statistics message.
    private: void OnStatsMsg(ConstWorldStatisticsPtr &_msg);

    /// internal
    /// \brief Update the vertex shader parameters.
    private: void SetShaderParams();

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<WavefieldVisualPluginPrivate> data;
  };

} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_VISUAL_PLUGIN_HH_
