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
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <ignition/math/Rand.hh>
#include <ros/ros.h>

namespace gazebo
{
  /// \brief A plugin that simulates a simple wind model. It accepts the
  /// following parameters:
  ///
  /// <bodyName>: The link that will receive the effect of the wind.
  /// <wind_direction>: Wind direction vector.
  /// <wind_coeff_vector>: Coefficients from Sarda et al.,
  /// <wind_mean_velocity>: The wind average velocity.
  /// <var_wind_gain_constants>: Variable wind speed gain constant.
  /// <var_wind_time_constants>: Variable wind speed time constant.
  /// <random_seed>: Set the seed for wind speed randomization.
  /// <update_rate>: Publishing rate of the wind topic. If set to 0, it will not
  /// publish, if set to a -1 it will publish every simulation iteration.
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

    /// \brief Pointer to the Gazebo world
    private: physics::WorldPtr world;

    /// \brief Pointer to model link in gazebo,
    ///  optionally specified by the bodyName parameter,
    ///  The states are taken from this link and forces applied to this link.
    private: physics::LinkPtr link;

    /// \brief Wind velocity unit vector in Gazebo coordinates [m/s].
    private: ignition::math::Vector3d windDirection;

    /// \brief Wind force coefficients.
    private: ignition::math::Vector3d windCoeff;

    /// \brief Average wind velocity.
    private: double windMeanVelocity;

    /// \brief Gain constant.
    private: double gainConstant;

    /// \brief Time constant.
    private: double timeConstant;

    /// \brief Previous time.
    private: double previousTime;

    /// \brief Velocity at previous time.
    private: double previousVarVel;

    /// \brief ROS node handle.
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief Publisher for wind speed.
    private: ros::Publisher windSpeedPub;

    /// \brief Publisher for wind direction.
    private: ros::Publisher windDirectionPub;

    /// \brief Topic where the wind speed is published.
    private: std::string topicWindSpeed = "/wind_speed";

    /// \brief Topic where the wind direction is published.
    private: std::string topicWindDirection = "/wind_direction";

    /// \brief Last time wind speed and direction was published.
    private: double lastPublishTime = 0;

    /// \brief Update rate buffer for wind speed and direction.
    private: double updateRate;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;
  };
}

#endif
