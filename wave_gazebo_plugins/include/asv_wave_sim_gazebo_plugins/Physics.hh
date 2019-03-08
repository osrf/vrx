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

/// \file Physics.hh
/// \brief This file contains definitions for the physics models.

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_PHYSICS_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_PHYSICS_HH_

namespace asv
{

///////////////////////////////////////////////////////////////////////////////
// Physics

  /// \brief A collection of static methods for various physics calculations.
  class Physics
  {

    /// \brief Compute the deep water dispersion.
    ///
    /// \param[in] _wavenumber  The wavenumber: k = 2 PI / wavelength.
    /// \return                 The angular frequency omega.
    public: static double DeepWaterDispersionToOmega(double _wavenumber);

    /// \brief Compute the deep water dispersion.
    ///
    /// \param[in] _omega       The angular frequency: omega = 2 PI / T.
    /// \return                 The wavenumber k.
    public: static double DeepWaterDispersionToWavenumber(double _omega);
  };
  
} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_PHYSICS_HH_
