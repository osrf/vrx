/*
 * Copyright (C) 2017  Brian Bingham
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

#ifndef USV_GAZEBO_PLUGINS_WIND_HH_
#define USV_GAZEBO_PLUGINS_WIND_HH_

#include <ros/ros.h>
#include <memory>
#include <string>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <ignition/math/Rand.hh>

namespace gazebo
{
  /// \brief A plugin that simulates a simple wind model. It accepts the
  /// following parameters:
  ///
  /// <bodyName>: The link that will receive the effect of the wind.
  ///
  /// <wind_direction>: Wind direction vector. Wind direction is specified as
  /// the positive direction of the wind velocity vector in the horizontal plane
  /// in degrees using the ENU coordinate convention
  ///
  /// <wind_coeff_vector>: Coefficients from Sarda et al.,
  ///
  /// <wind_mean_velocity>: The wind average velocity.
  ///
  /// <var_wind_gain_constants>: Variable wind speed gain constant.
  ///
  /// <var_wind_time_constants>: Variable wind speed time constant.
  ///
  /// <random_seed>: Set the seed for wind speed randomization.
  ///
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
