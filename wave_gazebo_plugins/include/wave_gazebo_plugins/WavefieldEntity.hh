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

/// \file WavefieldEntity.hh
/// \brief This file contains the definition for a Gazebo physics object
/// that allows a wave field to be added into a simulated world.

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_ENTITY_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_ENTITY_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Base.hh>
#include <memory>

namespace asv
{
  
///////////////////////////////////////////////////////////////////////////////
// WavefieldEntity

  class WaveParameters;

  /// \internal
  /// \brief Class to hold private data for WavefieldEntity.
  class WavefieldEntityPrivate;

  /// \brief A class to manage a wave field that can be accessed from the World.
  class GZ_PHYSICS_VISIBLE WavefieldEntity : public gazebo::physics::Base
  {
    /// \brief Destructor.
    public: virtual ~WavefieldEntity();

    /// \brief Constructor.
    public: explicit WavefieldEntity(gazebo::physics::BasePtr _parent);

    /// \brief Load.
    public: virtual void Load(sdf::ElementPtr _sdf);

    /// \brief Finialize the object.
    public: virtual void Fini();

    /// \brief Initialize the object.
    public: virtual void Init();

    /// \brief Reset the object.
    public: virtual void Reset();

    /// \brief Update the object.
    public: virtual void Update();
    
    /// \brief Get a pointer to the wave pararameters.
    std::shared_ptr<const WaveParameters> GetWaveParams() const;

    /// \brief Make a wave field entity name given a parent object name.
    ///
    /// \param[in] _parentName  The name of the parent object.
    /// \return                 The name of the wave field entity.
    public: static std::string MakeName(const std::string& _parentName);

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<WavefieldEntityPrivate> data;
  };

} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_ENTITY_HH_
