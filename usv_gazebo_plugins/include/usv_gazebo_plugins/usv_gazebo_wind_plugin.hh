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

#include <vector>
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
  /// <models_n>: the number of modelsto be effected by the wind.
  /// <model_name_i> where 0<=i<models_n, name of model i
  /// <link_name_i> where 0<=i<models_n, name of link to be effected by wind on model i
  /// <coeff_vector_i> where 0<=i<models_n, the wind coeffcient vector for the link on model i
  ///
  /// <wind_direction>: Wind direction vector. Wind direction is specified as
  /// the positive direction of the wind velocity vector in the horizontal plane
  /// in degrees using the ENU coordinate convention
  ///
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
  class UsvWindPlugin : public WorldPlugin
  {
    struct WindObj
    {
      /// \Bool to show weather the model and link pointers have been set
      bool init=false;
      /// \name of model as it will be looked by in the world
      std::string model_name;
      /// \model Pointer to the model
      physics::ModelPtr model;
      /// \Name of the link on that model
      std::string link_name;
      /// \brief Pointer to model link in gazebo,
      ///  optionally specified by the bodyName parameter,
      ///  The states are taken from this link and forces applied to this link.
      physics::LinkPtr link;
      /// \brief Wind force coefficients.
      ignition::math::Vector3d windCoeff;
    };
    /// \brief Constructor.
    public: UsvWindPlugin();

    /// \brief Destructor.
    public: virtual ~UsvWindPlugin() = default;

    // Documentation inherited.
    public: virtual void Load(physics::WorldPtr _parent,
                              sdf::ElementPtr _sdf);

    /// \brief Callback executed at every physics update.
    protected: virtual void Update();

    /// \breif vector of simple objects effected by the wind
    private: std::vector<UsvWindPlugin::WindObj> windObjs;

    /// \breif Bool to keep track if ALL of the windObjs have been initialized
    private: bool windObjsInit = false;
 
    /// \brief Pointer to the Gazebo world
    private: physics::WorldPtr world;

    /// \brief Wind velocity unit vector in Gazebo coordinates [m/s].
    private: ignition::math::Vector3d windDirection;

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
