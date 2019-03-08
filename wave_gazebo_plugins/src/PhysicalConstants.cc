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

#include "asv_wave_sim_gazebo_plugins/PhysicalConstants.hh"

namespace asv 
{

///////////////////////////////////////////////////////////////////////////////    
// PhysicalConstants

  double PhysicalConstants::Gravity()
  {
    return -9.8;
  }

  double PhysicalConstants::G()
  {
    return 6.67408E-11;
  }

  double PhysicalConstants::WaterDensity()
  {
    return 998.6;
  }

  double PhysicalConstants::WaterKinematicViscosity()
  {
    return 1.0533E-6;
  }

} // namespace asv

