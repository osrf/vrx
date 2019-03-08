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

/// \file PhysicalConstants.hh
/// \brief This file contains definitions of some physical constants used 
/// in the physics calculations.

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_PHYSICAL_CONSTANTS_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_PHYSICAL_CONSTANTS_HH_

namespace asv
{

///////////////////////////////////////////////////////////////////////////////
// PhysicalConstants

  /// \brief A collection of static methods to retrieve physical constants.
  class PhysicalConstants
  {
    /// \brief Uniform acceleration due to gravity at earth's surface (orientation is z-up).
    ///
    /// \return     -9.8 [m s-2].
    public: static double Gravity(); 
    
    /// \brief Universal gravitational constant.
    ///
    /// \return     6.67408E-11 [m3 kg-1 s-2].
    public: static double G();
    
    /// \brief Density of water.
    ///
    /// \return     998.6 [kg m-3].
    public: static double WaterDensity();

    /// \brief Kinematic viscosity of water at 18 dgree C.
    ///
    /// Source:
    /// <https://www.engineeringtoolbox.com/water-dynamic-kinematic-viscosity-d_596.html>
    ///
    /// \return     1.0533E-6 [m2 s-1].
    public: static double WaterKinematicViscosity();
  };

} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_PHYSICAL_CONSTANTS_HH_
