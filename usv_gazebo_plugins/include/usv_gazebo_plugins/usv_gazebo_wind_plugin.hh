/*

Copyright (c) 2018, Brian Bingham
All rights reserved

This file is part of the usv_gazebo_dynamics_plugin package, known as this
Package.

This Package free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This Package s distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this package.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef USV_GAZEBO_PLUGINS_WIND_HH_
#define USV_GAZEBO_PLUGINS_WIND_HH_

#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief A plugin that simulates a simple wind model. It accepts the
  /// following parameters:
  ///
  /// <bodyName>: The link that will receive the effect of the wind.
  /// <wind_velocity_vector>: The wind vector.
  /// <wind_coefficient_vector>: Coefficients from Sarda et al.,
  /// "Station-keeping control of an unmanned surface vehicle exposed to
  /// current and wind disturbances".
  class UsvWindPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: UsvWindPlugin();

    /// \brief Destructor.
    public: virtual ~UsvWindPlugin() = default;

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _parent,
                              sdf::ElementPtr _sdf);

    /// \brief Callback executed at every physics update.
    protected: virtual void Update();

    /// \brief Pointer to model link in gazebo,
    ///  optionally specified by the bodyName parameter,
    ///  The states are taken from this link and forces applied to this link.
    private: physics::LinkPtr link;

    /// \brief Wind velocity in Gazebo coordinates [m/s].
    private: math::Vector3 windVelocity;

    /// \brief Wind force coefficients.
    private: math::Vector3 windCoeff;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;
  };
}

#endif
